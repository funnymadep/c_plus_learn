cmake_minimum_required(VERSION 2.8.3)
project(pcd_publisher)

add_compile_options(-std=c++14)

set(OpenCV_DIR /woonghyun/opencv/opencv-3.4.0/build/unix-install)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcd_publisher src/pcd_publisher.cpp)

target_link_libraries(pcd_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

