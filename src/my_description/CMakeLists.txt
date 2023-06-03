cmake_minimum_required(VERSION 3.0.2)
project(my_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_description
#  CATKIN_DEPENDS urdf xacro
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


