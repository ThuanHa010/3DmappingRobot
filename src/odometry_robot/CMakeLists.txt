cmake_minimum_required(VERSION 3.0.2)
project(odometry_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES odometry_robot
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(odom_publisher src/odometry.cpp)

target_link_libraries(odom_publisher
  ${catkin_LIBRARIES}
)


