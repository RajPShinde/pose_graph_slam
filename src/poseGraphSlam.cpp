#include <poseGraphSlam.hpp>

PoseGraphSLAM::PoseGraphSLAM(ros::NodeHandle &nodeHandle) : nh_(nodeHandle), tfListener_(tfBuffer_){ 
    
    setupOptimizer();

    // Subscribers  
    scanSub_ = nh_.subscribe("/scan", 1, &PoseGraphSLAM::scanCallback, this);

    // Publishers
    mapPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/slam/map",2);
    slamMarkersPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/slam",1);
}

PoseGraphSLAM::~PoseGraphSLAM(){
}

void PoseGraphSLAM::scanCallback(const sensor_msgs::LaserScan msg){    
    // Get LiDAR Scan
    sensor_msgs::PointCloud2 scan;
    scan.header = msg.header;

    projector_.projectLaser(msg, scan);
    pcl::fromROSMsg(scan, *cloud_);

    if (!cloud_->is_dense){ 
        std::vector<int> indices; 
        pcl::removeNaNFromPointCloud(*cloud_,*cloud_, indices); 
    }

    cloud_->points.erase(
    std::remove_if(cloud_->points.begin(), cloud_->points.end(),
                    [](const pcl::PointXYZ& point) {
                    return (point.getVector3fMap().squaredNorm() < 1e-3);
                    }),cloud_->points.end());

    // Update the cloud size
    cloud_->width = static_cast<uint32_t>(cloud_->points.size());
    cloud_->height = 1;

    // Get LiDAR Pose
    geometry_msgs::TransformStamped transformStamped;
    double x, y, z;
    double roll, pitch, yaw;
    try{
        transformStamped = tfBuffer_.lookupTransform("odom", msg.header.frame_id, msg.header.stamp, ros::Duration(1));
        x = transformStamped.transform.translation.x;
        y = transformStamped.transform.translation.y;

        tf::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        odom_ = {x, y, yaw};
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
    }

    if(!dataAvailable_ && odom_ != Pose(INT_MAX, INT_MAX, INT_MAX))
        dataAvailable_ = true;
}

void PoseGraphSLAM::setupOptimizer(){
    optimizer_.setVerbose(false);

    // Linear solver using dense cholesky decomposition
    auto linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();
    
    // Implementation of the Levenberg Algorithm
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));
    
    optimizer_.setAlgorithm(solver);
    ROS_INFO_STREAM("Optimizer Setup Done");
}

bool PoseGraphSLAM::align(fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr &source, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, Eigen::Matrix4f &transform, Eigen::Matrix3d &informationMatrix){
    
    // Voxelize based on sensor noise
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(0.1f, 0.1f, 0.1f);

    voxel.setInputCloud(source);
    pcl::PointCloud<pcl::PointXYZ>::Ptr srcFiltered(new pcl::PointCloud<pcl::PointXYZ>());
    voxel.filter(*srcFiltered);

    voxel.setInputCloud(target);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetFiltered(new pcl::PointCloud<pcl::PointXYZ>());
    voxel.filter(*targetFiltered);

    // Set cloud
    registration->setInputSource(srcFiltered);
    registration->setInputTarget(targetFiltered);

    // Set Registeration params
    registration->setMaximumIterations(1000);
    registration->setMaxCorrespondenceDistance(0.4);
    registration->setTransformationEpsilon(1e-6);
    registration->setEuclideanFitnessEpsilon(1e-6);

    // // Add a distance-based rejector
    // pcl::registration::CorrespondenceRejectorDistance::Ptr rejDist(new pcl::registration::CorrespondenceRejectorDistance);
    // rejDist->setMaximumDistance(0.2);
    // registration->getCorrespondenceRejectors().push_back(rejDist);

    // Add RANSAC-based rejector
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>::Ptr rejRansac(new pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>);
    rejRansac->setInlierThreshold(0.05);
    rejRansac->setMaximumIterations(1000);
    rejRansac->setInputSource(source);
    rejRansac->setInputTarget(target);
    registration->getCorrespondenceRejectors().push_back(rejRansac);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

    registration->align(*aligned, transform);

    if(!registration->hasConverged()){
        ROS_WARN("FGICP has not converged");
        return false;
    }

    transform = registration->getFinalTransformation().cast<float>();

    double score = registration->getFitnessScore();

    ROS_INFO_STREAM("FGICP has converged with a score: " << score);

    // Generate Information Matrix
    Eigen::MatrixXd hessianSE3 = registration->getFinalHessian();
    Eigen::Matrix3d hessian;
    hessian(0,0) = hessianSE3(0,0);  // x-x
    hessian(0,1) = hessianSE3(0,1);  // x-y
    hessian(0,2) = hessianSE3(0,5);  // x-yaw
    hessian(1,0) = hessianSE3(1,0);  // y-x
    hessian(1,1) = hessianSE3(1,1);  // y-y
    hessian(1,2) = hessianSE3(1,5);  // y-yaw
    hessian(2,0) = hessianSE3(5,0);  // yaw-x
    hessian(2,1) = hessianSE3(5,1);  // yaw-y
    hessian(2,2) = hessianSE3(5,5);  // yaw-yaw
    hessian *= 1.0 / (score + 1e-6);

    informationMatrix = hessian;

    return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PoseGraphSLAM::concatinate(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &p){
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>());
    for(int i = 0; i < p.size(); i++){
        *merged += *p[i];
    }
    return merged;
}

double PoseGraphSLAM::wrapPi(double angle){
    while(angle > M_PI)
        angle -= 2.0 * M_PI;
    while(angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

int PoseGraphSLAM::isLoopClosurePossible(const int id){
    if(allVertex.size() - 1 < loopClosureMinVerticesThreshold_)
        return -1;
    
    auto currentPose = allVertex[id].vertex->estimate().toVector();

    for(int i = 0; i < allVertex.size() - 1 - loopClosureMinVerticesThreshold_; i++){
        auto vertexPose = allVertex[i].vertex->estimate().toVector();
        double dX = currentPose(0) - vertexPose(0); 
        double dY = currentPose(1) - vertexPose(1);
        if(std::sqrt(std::pow(dX, 2) + std::pow(dY, 2)) <= loopClosureDistanceThreshold_)
            return i;
    }

    return -1;
}

void PoseGraphSLAM::addVertex(int id, Eigen::Matrix4f pose, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool fixed){
    // Add a vertex
    ROS_INFO_STREAM("Adding Vertex: " << id);

    g2o::VertexSE2 *vertex = new g2o::VertexSE2();

    vertex->setId(id);
    vertex->setEstimate(g2o::SE2(pose(0, 3), pose(1, 3), std::atan2(pose(1, 0), pose(0, 0))));
    vertex->setFixed(fixed);
    
    optimizer_.addVertex(vertex);

    allVertex.push_back(Vertex(id, cloud, vertex));
}

void PoseGraphSLAM::addEdge(int id1, int id2, Eigen::Matrix4f transform, Eigen::Matrix3d informationMatrix){
    ROS_INFO_STREAM("Adding Edge: " << id1 << " - " << id2);

    g2o::EdgeSE2 *edge = new g2o::EdgeSE2();
    
    edge->setVertex(0, allVertex[id1].vertex);
    edge->setVertex(1, allVertex[id2].vertex);

    // Orthonormalize the rotation
    Eigen::Matrix2d R = transform.block<2,2>(0,0).cast<double>();
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = svd.matrixU() * svd.matrixV().transpose();
    double yaw = std::atan2(R(1,0), R(0,0));

    edge->setMeasurement(g2o::SE2(transform(0, 3), transform(1, 3), yaw));
    
    edge->setInformation(informationMatrix);

    if(useKernel_){
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber();
        rk->setDelta(kernelDelta_);
        edge->setRobustKernel(rk);
    }

    optimizer_.addEdge(edge);

    allEdges_.push_back(Edge(id1, id2, allVertex[id1].vertex, allVertex[id2].vertex));
}

void PoseGraphSLAM::optimize(){
    ROS_INFO_STREAM("Optimizing Graph");
    optimizer_.initializeOptimization();
    optimizer_.optimize(100);
}

void PoseGraphSLAM::run(){
    ros::Rate loop(2);

    while(ros::ok()){
        ros::spinOnce();

        if(!dataAvailable_)
            continue;
        
        if(prevOdom_ == Pose(INT_MAX, INT_MAX, INT_MAX)){
            prevOdom_ = odom_;
            pose_ << std::cos(odom_.yaw), -std::sin(odom_.yaw), 0, odom_.x,
                     std::sin(odom_.yaw),  std::cos(odom_.yaw), 0, odom_.y,
                     0            ,  0              , 1, 0,
                     0            ,  0              , 0, 1;
            // Add first vertex that is fixed
            addVertex(id_, pose_, cloud_, true);
            continue;
        }

        // Check if change in wheel odometry is above thresholds
        float dX = odom_.x - prevOdom_.x;
        float dY = odom_.y - prevOdom_.y;
        float dYaw = wrapPi(odom_.yaw - prevOdom_.yaw);

        if(std::sqrt(std::pow(dX, 2) + std::pow(dY, 2)) >= translationThreshold_ || std::abs(dYaw) >= rotationThreshold_){

            // Rotation of previous odometry
            float cosYaw = std::cos(-prevOdom_.yaw);
            float sinYaw = std::sin(-prevOdom_.yaw);

            // Express translation in the robot's local (previous) frame
            float local_dx =  cosYaw * dX - sinYaw * dY;
            float local_dy =  sinYaw * dX + cosYaw * dY;

            ROS_INFO_STREAM(local_dx<<" "<<local_dy<<" "<<dYaw);

            id_++;

            // Find transform between previous vertex and new vertex(current scan)
            pcl::PointCloud<pcl::PointXYZ>::Ptr a, b;
            a = allVertex[id_ - 1].cloud;
            b = cloud_;

            Eigen::Matrix4f transform;
            transform << std::cos(dYaw), -std::sin(dYaw), 0, local_dx,
                         std::sin(dYaw),  std::cos(dYaw), 0, local_dy,
                         0            ,  0              , 1, 0,
                         0            ,  0              , 0, 1;

            Eigen::Matrix4f odomTransform = transform;
            
            fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>::Ptr fastGICP(new fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>());
            Eigen::Matrix3d informationMatrix;
            if(!align(fastGICP, b, a, transform, informationMatrix)){
                id_--;
                continue;
            }

            // Current pose of the robot
            pose_ = pose_ * transform;

            // ROS_INFO_STREAM("\n" << transform);
            // ROS_INFO_STREAM("\n" << pose_);

            // Add new vertex
            addVertex(id_, pose_, cloud_, false);

            // Add new edge for sacn matching data
            addEdge(id_ - 1, id_, transform, informationMatrix);


            // Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            // double sigma_trans = 0.01;
            // double sigma_rot   = 0.01;
            // covariance(0,0) = sigma_trans*sigma_trans;
            // covariance(1,1) = sigma_trans*sigma_trans;
            // covariance(2,2) = sigma_rot*sigma_rot;
            // Eigen::Matrix3d odomInformation = covariance.inverse();
            // // Add new edge for Odometry data
            // addEdge(id_ - 1, id_, odomTransform, odomInformation);

            prevOdom_ = odom_;

            // Check if Loop Closure is available
            int loopClosureId = isLoopClosurePossible(id_);
            if(loopClosureId != -1){
                ROS_INFO_STREAM("Performing Loop Closure between: " << loopClosureId << " - " << id_);

                pcl::PointCloud<pcl::PointXYZ>::Ptr a, b;
                a = allVertex[loopClosureId].cloud;
                b = cloud_;

                auto estimate1 = allVertex[id_].vertex->estimate().toVector();
                auto estimate2 = allVertex[loopClosureId].vertex->estimate().toVector();

                double dX = estimate1(0) - estimate2(0);
                double dY = estimate1(1) - estimate2(1);
                double dYaw = wrapPi(estimate1(2) - estimate2(2));

                // Rotation of previous odometry
                float cosYaw = std::cos(-estimate2(2));
                float sinYaw = std::sin(-estimate2(2));

                // Express translation in the robot's local (previous) frame
                float local_dx =  cosYaw * dX - sinYaw * dY;
                float local_dy =  sinYaw * dX + cosYaw * dY;

                Eigen::Matrix4f transform;
                transform << std::cos(dYaw), -std::sin(dYaw), 0, local_dx,
                             std::sin(dYaw),  std::cos(dYaw), 0, local_dy,
                             0            ,  0              , 1, 0,
                             0            ,  0              , 0, 1;

                fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>::Ptr fastGICP(new fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>());
                Eigen::Matrix3d informationMatrix;
                if(align(fastGICP, b, a, transform, informationMatrix)){
                    addEdge(loopClosureId, id_, transform, informationMatrix);
                    optimize();
                }
            }

            visualize();
        }
        loop.sleep();
    }
}

void PoseGraphSLAM::visualize(){

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>());

    for(int i = 0; i < allVertex.size(); i++){
        auto estimate = allVertex[i].vertex->estimate().toVector();
        pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>(*allVertex[i].cloud));
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
        
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << static_cast<float>(estimate(0)), static_cast<float>(estimate(1)), 0.0f;
        transform.rotate(Eigen::AngleAxisf(static_cast<float>(estimate(2)), Eigen::Vector3f::UnitZ()));

        pcl::transformPointCloud(*input, *output, transform);

        *merged += *output;
    }

    sensor_msgs::PointCloud2 mapCloud;
    pcl::toROSMsg(*merged, mapCloud);
    mapCloud.header.frame_id = "odom";
    mapCloud.header.stamp = ros::Time::now();
    mapPub_.publish(mapCloud);

    visualization_msgs::MarkerArray slamMarkers;

    int vertexMarkerId = 0;
    for(int i = 0; i < allVertex.size(); i++){

        auto estimate = allVertex[i].vertex->estimate().toVector();

		visualization_msgs::Marker vertexMarker;
		vertexMarker.header.frame_id = "odom";
		vertexMarker.type = visualization_msgs::Marker::SPHERE;
		vertexMarker.action = visualization_msgs::Marker::ADD;
		vertexMarker.ns = "/vertex_markers";
		vertexMarker.id = vertexMarkerId++;

		vertexMarker.scale.x = 0.1;
		vertexMarker.scale.y = 0.1;
		vertexMarker.scale.z = 0.1;

		vertexMarker.color.r = 0;
		vertexMarker.color.g = 0;
		vertexMarker.color.b = 1;
		vertexMarker.color.a = 1;

      	vertexMarker.pose.position.x = estimate(0);
      	vertexMarker.pose.position.y = estimate(1);
      	vertexMarker.pose.position.z = 0;

		vertexMarker.pose.orientation.x = vertexMarker.pose.orientation.w = vertexMarker.pose.orientation.z = 0;
		vertexMarker.pose.orientation.w = 1;

        slamMarkers.markers.push_back(vertexMarker);
    }

    int edgeMarkerId = 0;
    for(int i = 0; i < allEdges_.size(); i++){

        auto estimate1 = allEdges_[i].vertex1->estimate().toVector();
        auto estimate2 = allEdges_[i].vertex2->estimate().toVector();

		visualization_msgs::Marker edgeMarker;
		edgeMarker.header.frame_id = "odom";
		edgeMarker.type = visualization_msgs::Marker::LINE_STRIP;
		edgeMarker.action = visualization_msgs::Marker::ADD;
		edgeMarker.ns = "/edge_markers";
		edgeMarker.id = edgeMarkerId++;

		edgeMarker.scale.x = 0.01;

		edgeMarker.color.r = 0;
		edgeMarker.color.g = 0;
		edgeMarker.color.b = 0;
		edgeMarker.color.a = 1;

        geometry_msgs::Point point;
        point.x = estimate1(0);
        point.y = estimate1(1);
        edgeMarker.points.push_back(point);
        point.x = estimate2(0);
        point.y = estimate2(1);
        edgeMarker.points.push_back(point);

        slamMarkers.markers.push_back(edgeMarker);
    }

    if(allVertex.size()){

        auto estimate = allVertex.back().vertex->estimate().toVector();

        visualization_msgs::Marker loopClosureDiscMarker;
        loopClosureDiscMarker.header.frame_id = "odom";
        loopClosureDiscMarker.type = visualization_msgs::Marker::CYLINDER;
        loopClosureDiscMarker.action = visualization_msgs::Marker::ADD;
        loopClosureDiscMarker.ns = "/loop_closure_disc_markers";
        loopClosureDiscMarker.id = 0;

        loopClosureDiscMarker.scale.x = loopClosureDistanceThreshold_*2;
        loopClosureDiscMarker.scale.y = loopClosureDistanceThreshold_*2;
        loopClosureDiscMarker.scale.z = 0.01;

        loopClosureDiscMarker.color.r = 1;
        loopClosureDiscMarker.color.g = 1;
        loopClosureDiscMarker.color.b = 0;
        loopClosureDiscMarker.color.a = 0.3;

        loopClosureDiscMarker.pose.position.x = estimate(0);
        loopClosureDiscMarker.pose.position.y = estimate(1);
        loopClosureDiscMarker.pose.position.z = 0;

        loopClosureDiscMarker.pose.orientation.x = loopClosureDiscMarker.pose.orientation.w = loopClosureDiscMarker.pose.orientation.z = 0;
        loopClosureDiscMarker.pose.orientation.w = 1;

        slamMarkers.markers.push_back(loopClosureDiscMarker);
    }


	slamMarkersPub_.publish(slamMarkers);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "poseGraphSLAM");
    ros::NodeHandle nodeHandle;

	PoseGraphSLAM slam(nodeHandle);    
    slam.run();

	return 0;
}

