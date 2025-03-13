/**
 * @file poseGraphSlam.hpp
 * @author Raj Shinde (raj.shinde@viabotinc.com)
 * @brief Pose Graph Slam
 * @version 1.1
 * @date 2022-08-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef INCLUDE_POSEGRAPHSLAM_HPP_
#define INCLUDE_POSEGRAPHSLAM_HPP_

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <nav_msgs/OccupancyGrid.h>
#include <numeric>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <laser_geometry/laser_geometry.h>
#include <tf2_ros/transform_listener.h>

#include <iostream>
#include <numeric>
#include <ctime>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <stdlib.h>
#include <mutex>
#include <shared_mutex>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/types_slam2d.h>

#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense>

#include <pcl_conversions/pcl_conversions.h>

class PoseGraphSLAM{
    public:

        PoseGraphSLAM(ros::NodeHandle &nodeHandle);

        ~PoseGraphSLAM();

        void scanCallback(const sensor_msgs::LaserScan msg);

        void setupOptimizer();

        static pcl::PointCloud<pcl::PointXYZI>::Ptr align(pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration, const pcl::PointCloud<pcl::PointXYZI>::Ptr &source, const pcl::PointCloud<pcl::PointXYZI>::Ptr &target, Eigen::Matrix4f &transform);

        static pcl::PointCloud<pcl::PointXYZI>::Ptr concatinate(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &p);

        void run();

    private:
        ros::NodeHandle nh;
        ros::Publisher mapPub_;
        ros::Subscriber scanSub_;
        laser_geometry::LaserProjection projector_;
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;

        std::shared_mutex callbackMutex;

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZI>()};
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr icp_;

        g2o::SparseOptimizer optimizer_;
};

#endif  //  INCLUDE_POSEGRAPHSLAM_HPP_