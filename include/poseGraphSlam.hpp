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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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
#include <g2o/core/robust_kernel_impl.h>

#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/correspondence_rejection.h>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/lsq_registration.hpp>


#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <Eigen/Dense>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/pcl_config.h>


struct Pose{
    double x;
    double y;
    double yaw;

    Pose(){};
    Pose(double a, double b, double c): x(a), y(b), yaw(c) {};

    bool operator == (const Pose &p) const
    {
        return (x == p.x && y == p.y && yaw == p.yaw);
    }

    bool operator != (const Pose &p) const
    {
        return (x != p.x || y != p.y || yaw != p.yaw);
    }
};

struct Vertex{
    int id;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    g2o::VertexSE2 *vertex;

    Vertex(){};
    Vertex(int a, pcl::PointCloud<pcl::PointXYZ>::Ptr b, g2o::VertexSE2 *c) {
        id = a;
        cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*b));
        vertex = c;
    }
};

struct Edge{
    int id1;
    int id2;
    g2o::VertexSE2 *vertex1;
    g2o::VertexSE2 *vertex2;

    Edge(){}
    Edge(int a, int b, g2o::VertexSE2 *c, g2o::VertexSE2 *d){
        id1 = a;
        id2 = b;
        vertex1 = c;
        vertex2 = d;
    }
};

class PoseGraphSLAM{
    public:

        PoseGraphSLAM(ros::NodeHandle &nodeHandle);

        ~PoseGraphSLAM();

        void scanCallback(const sensor_msgs::LaserScan msg);

        void setupOptimizer();

        static bool align(fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr &source, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, Eigen::Matrix4f &transform, Eigen::Matrix3d &informationMatrix);

        static pcl::PointCloud<pcl::PointXYZ>::Ptr concatinate(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &p);

        void addVertex(int id, Eigen::Matrix4f pose, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool fixed);

        void addEdge(int id1, int id2, Eigen::Matrix4f transform, Eigen::Matrix3d informationMatrix);

        void optimize();

        double wrapPi(double angle);

        int isLoopClosurePossible(const int id);

        void run();

        void visualize();

    private:
        ros::NodeHandle nh_;
        ros::Publisher mapPub_, slamMarkersPub_;
        ros::Subscriber scanSub_;
        laser_geometry::LaserProjection projector_;
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;

        std::shared_mutex callbackMutex;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_{new pcl::PointCloud<pcl::PointXYZ>()};

        g2o::SparseOptimizer optimizer_;
        bool useKernel_ = false;
        double kernelDelta_ = 1.0;

        Pose odom_{INT_MAX, INT_MAX, INT_MAX};
        Pose prevOdom_{INT_MAX, INT_MAX, INT_MAX};

        std::vector<Vertex> allVertex;
        std::vector<Edge> allEdges_;

        Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();

        int id_ = 0;

        bool dataAvailable_ = false;
        double translationThreshold_ = 1.0;
        double rotationThreshold_ = 0.2;
        double loopClosureDistanceThreshold_ = 3.5;
        int loopClosureMinVerticesThreshold_ = 20;
};

#endif  //  INCLUDE_POSEGRAPHSLAM_HPP_