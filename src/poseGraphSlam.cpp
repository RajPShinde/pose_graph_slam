#include <poseGraphSlam.hpp>

PoseGraphSLAM::PoseGraphSLAM(ros::NodeHandle &nodeHandle) : nh(nodeHandle), tfListener_(tfBuffer_){ 

}

PoseGraphSLAM::~PoseGraphSLAM(){
}

void PoseGraphSLAM::scanCallback(const sensor_msgs::LaserScan msg){    

}

void PoseGraphSLAM::setupOptimizer(){

}

pcl::PointCloud<pcl::PointXYZI>::Ptr PoseGraphSLAM::align(pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration, const pcl::PointCloud<pcl::PointXYZI>::Ptr &source, const pcl::PointCloud<pcl::PointXYZI>::Ptr &target, Eigen::Matrix4f &transform){

}

pcl::PointCloud<pcl::PointXYZI>::Ptr PoseGraphSLAM::concatinate(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &p){

}

void PoseGraphSLAM::run(){

}

int main(int argc, char **argv){
    ros::init(argc, argv, "poseGraphSLAM");
    ros::NodeHandle nodeHandle;

	PoseGraphSLAM slam(nodeHandle);    
    slam.run();

	return 0;
}

