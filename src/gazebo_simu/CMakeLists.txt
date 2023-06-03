cmake_minimum_required(VERSION 3.0.2)
project(gazebo_simu)


find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  my_description
  roscpp
  rospy
  sensor_msgs
  controller_manager
  joint_state_controller
  joy

  tf
  tf2_sensor_msgs
)

find_package(gazebo REQUIRED)

catkin_package()
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS})
