cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(3dLocalizer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (3dLocalizer_exe pcl_visualizer.cpp)
target_link_libraries (3dLocalizer_exe ${PCL_LIBRARIES})
