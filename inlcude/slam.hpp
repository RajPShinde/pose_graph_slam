#ifndef INCLUDE_SLAM_HPP_
#define INCLUDE_SLAM_HPP_

#include <ros/ros.h>
#include <cmath>
#include <iostream>

class SLAM
{
    public:

        SLAM(ros::NodeHandle &nh);

        ~SLAM();

        void callbackLaserScan();

        void callbackOdom();

        void run();

    private:


};

#endif  //  INCLUDE_SLAM_HPP_