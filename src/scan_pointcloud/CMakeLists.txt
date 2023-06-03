cmake_minimum_required(VERSION 3.0.2)
project(scan_pointcloud)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  tf
  pcl_ros
  pcl_conversions
  tf_conversions
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scan_pointcloud
#   CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf pcl_ros
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(scan2pcl_node src/scan2pcl3.cpp)

target_link_libraries(scan2pcl_node  ${catkin_LIBRARIES})


