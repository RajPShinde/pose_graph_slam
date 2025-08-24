#include <poseGraphSlam.hpp>

PoseGraphSLAM::PoseGraphSLAM(ros::NodeHandle &nodeHandle) : nh_(nodeHandle), tfListener_(tfBuffer_){ 
    
    setupOptimizer();
    setupRegisteration(icp_);

    // Subscribers  
    scanSub_ = nh_.subscribe("/scan/raw", 1, &PoseGraphSLAM::scanCallback, this);

    // Publishers
    mapPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/slam/map",2);
    slamMarkersPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/slam",1);

    std::cout << PCL_VERSION << std::endl;
}

PoseGraphSLAM::~PoseGraphSLAM(){
}

void PoseGraphSLAM::scanCallback(const sensor_msgs::LaserScan msg){    
    // Get LiDAR Scan
    sensor_msgs::PointCloud2 scan;
    scan.header = msg.header;

    projector_.projectLaser(msg, scan);
    pcl::fromROSMsg(scan, *cloud_);

    // Get LiDAR Pose
    geometry_msgs::TransformStamped transformStamped;
    double x, y, z;
    double roll, pitch, yaw;
    try{
        transformStamped = tfBuffer_.lookupTransform("odom", "laser_link", ros::Time(0));
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

void PoseGraphSLAM::setupRegisteration(pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration){
  	registration->setTransformationEpsilon(0.01);
	registration->setMaximumIterations(10000);
    registration->setMaxCorrespondenceDistance(0.2);
    ROS_INFO_STREAM("Registration Setup Done");
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PoseGraphSLAM::align(pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr &source, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, Eigen::Matrix4f &transform){
    registration->setInputSource(source);
    registration->setInputTarget(target);
    registration->setMaximumIterations(1000);
    // registration->setMaxCorrespondenceDistance(0.01);
    // registration->setTransformationEpsilon(0.01);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
    registration->align(*aligned, transform);
    transform = registration->getFinalTransformation();
    return aligned;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PoseGraphSLAM::concatinate(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &p){
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>());
    for(int i = 0; i < p.size(); i++){
        *merged += *p[i];
    }
    return merged;
}

void PoseGraphSLAM::addVertex(int id, Eigen::Matrix4f pose, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool fixed){
    // Add a vertex
    ROS_INFO_STREAM("Adding vertex " << id);

    g2o::VertexSE2 *vertex = new g2o::VertexSE2();

    vertex->setId(id);
    vertex->setEstimate(g2o::SE2(pose(0, 3), pose(1, 3), std::atan2(pose(1, 0), pose(0, 0))));
    vertex->setFixed(fixed);
    
    optimizer_.addVertex(vertex);

    allVertex.push_back(Vertex(id, cloud, vertex));
}

void PoseGraphSLAM::addEdge(int id1, int id2, Eigen::Matrix4f transform, Eigen::Matrix3d informationMatrix){
    ROS_INFO_STREAM("Adding edge " << id1 << " " << id2);

    g2o::EdgeSE2 *edge = new g2o::EdgeSE2();
    
    edge->setVertex(0, allVertex[id1].vertex);
    edge->setVertex(1, allVertex[id2].vertex);
    edge->setMeasurement(g2o::SE2(transform(0, 3), transform(1, 3), std::atan2(transform(1, 0), transform(0, 0))));
    
    edge->setInformation(informationMatrix);
    optimizer_.addEdge(edge);

    allEdges_.push_back(Edge(id1, id2, allVertex[id1].vertex, allVertex[id2].vertex));
}

void PoseGraphSLAM::optimize(){
    ROS_INFO_STREAM("Optimizing Graph");
    optimizer_.initializeOptimization();
    optimizer_.optimize(10);
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


void PoseGraphSLAM::run(){
    ros::Rate loop(10);

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
            // Add first fixed vertex
            addVertex(id_, pose_, cloud_, true);
            continue;
        }

        // Check if change in wheel odometry is above thresholds
        float dX = odom_.x - prevOdom_.x;
        float dY = odom_.y - prevOdom_.y;
        float dYaw = wrapPi(odom_.yaw - prevOdom_.yaw);

        if(std::sqrt(std::pow(dX, 2) + std::pow(dY, 2)) >= translationThreshold_ || std::abs(dYaw) >= rotationThreshold_){
            // ROS_INFO_STREAM(dX<<" "<<dY<<" "<<dYaw);
            // ROS_INFO_STREAM(odom_.yaw<<" "<<prevOdom_.yaw<<" "<<wrapPi(dYaw));

            prevOdom_ = odom_;

            id_++;

            // Find transform between previous vertex and new vertex(current scan)
            pcl::PointCloud<pcl::PointXYZ>::Ptr a, b;
            a = allVertex[id_ - 1].cloud;
            b = cloud_;

            Eigen::Matrix4f transform;
            transform << std::cos(dYaw), -std::sin(dYaw), 0, dX,
                         std::sin(dYaw),  std::cos(dYaw), 0, dY,
                         0            ,  0              , 1, 0,
                         0            ,  0              , 0, 1;

            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp(new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
            align(icp, b, a, transform);

            if(icp->hasConverged()){
                ROS_INFO_STREAM("ICP has converged, score is " << icp->getFitnessScore ());
                transform = icp->getFinalTransformation().cast<float>();
            }
            else{
                ROS_WARN("ICP has not converged.");
                continue;
            }

            // Current pose of the robot
            pose_ = pose_ * transform;

            // ROS_INFO_STREAM("\n" << transform);
            // ROS_INFO_STREAM("\n" << pose_);

            // Add new vertex
            addVertex(id_, pose_, cloud_, false);

            // Add new edge
            Eigen::Matrix3d informationMatrix = Eigen::Matrix3d::Identity() * 10;
            addEdge(id_ - 1, id_, transform, informationMatrix);

            // Check if Loop Closure is available
            int loopClousreId = isLoopClosurePossible(id_);
            if(loopClousreId != -1){
                ROS_WARN_STREAM("Current Id- " << id_ << ", Loop Closure Id- " << loopClousreId);
                ROS_INFO_STREAM("Performing Loop Closure");

                pcl::PointCloud<pcl::PointXYZ>::Ptr a, b;
                a = allVertex[loopClousreId].cloud;
                b = cloud_;
                
                pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp(new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
                align(icp, b, a, transform);

                if(icp->hasConverged()){
                    ROS_INFO_STREAM("ICP has converged, score is " << icp->getFitnessScore ());
                    transform = icp->getFinalTransformation().cast<float>();
                    // ROS_INFO_STREAM("\n" << transform);
                }
                else{
                    ROS_WARN("ICP has not converged.");
                    continue;
                }

                Eigen::Matrix3d informationMatrix = Eigen::Matrix3d::Identity() * 100;
                addEdge(100, 100, transform, informationMatrix);
                optimize();
            }

            visualize();
        }
        loop.sleep();
    }
}

void PoseGraphSLAM::visualize(){

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>());

    // ROS_INFO_STREAM("Visualizing Merged Cloud using " << allVertex.size() << " vertices");
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
		vertexMarker.lifetime = ros::Duration(10);
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
		edgeMarker.lifetime = ros::Duration(10);
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


	slamMarkersPub_.publish(slamMarkers);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "poseGraphSLAM");
    ros::NodeHandle nodeHandle;

	PoseGraphSLAM slam(nodeHandle);    
    slam.run();

	return 0;
}

