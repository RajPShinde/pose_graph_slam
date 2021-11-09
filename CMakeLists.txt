cmake_minimum_required(VERSION 3.2.1)
project (graph_slam)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
  )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)


add_executable(graphslam src/main.cpp src/slam.cpp src/graphOptimization.cpp src/iterativeClosestPoint.cpp include/slam.hpp include/graphOptimization.hpp include/iterativeClosestPoint.hpp)
target_link_libraries(graphslam ${catkin_LIBRARIES})
