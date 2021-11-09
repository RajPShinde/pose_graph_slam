#include <slam.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "GraphSLAM");
    ros::NodeHandle nh;
    SLAM start(nh);
    ros::Rate loop(10);
    while(ros::ok()){
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}