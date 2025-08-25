cmake_minimum_required(VERSION 3.10.0)
project(pose_graph_slam)

# Compiler Flags
set(CMAKE_CXX_STANDARD 17)

# Use -O3 Optimizations
set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  tf
  tf2
  sensor_msgs
  nav_msgs
  std_srvs
  laser_geometry
)

catkin_package(
 INCLUDE_DIRS include
)

find_package(roslaunch)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(g2o REQUIRED)
find_package(fast_gicp REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIR}
    ${G2O_INCLUDE_DIRS}
    ${fast_gicp_INCLUDE_DIRS}
)

link_directories(${G2O_LIBRARY_DIRS})

add_executable(poseGraphSlam src/poseGraphSlam.cpp)
target_link_libraries(poseGraphSlam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fast_gicp_LIBRARIES}
  g2o_core
  g2o_stuff
  g2o_types_slam2d
  g2o_types_slam3d
  g2o_solver_dense
  g2o_solver_csparse
  g2o_solver_cholmod
  g2o_solver_pcg)
add_dependencies(poseGraphSlam ${catkin_EXPORTED_TARGETS})


