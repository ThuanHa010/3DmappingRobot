cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ProcessPCD)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (detectObject detectObject.cpp)
target_link_libraries (detectObject ${PCL_LIBRARIES})

add_executable (improveMap improveMap.cpp)
target_link_libraries (improveMap ${PCL_LIBRARIES})

add_executable (improveMapSIMU improveMapSIMU.cpp)
target_link_libraries (improveMapSIMU ${PCL_LIBRARIES})

add_executable (view view.cpp)
target_link_libraries (view ${PCL_LIBRARIES})

add_executable (view2 view2.cpp)
target_link_libraries (view2 ${PCL_LIBRARIES})

add_executable (view_half view_half.cpp)
target_link_libraries (view_half ${PCL_LIBRARIES})

add_executable (mergeMap mergeMap.cpp)
target_link_libraries (mergeMap ${PCL_LIBRARIES})

add_executable (calcDistance calcDistance.cpp)
target_link_libraries (calcDistance ${PCL_LIBRARIES})