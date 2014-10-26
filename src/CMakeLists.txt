cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(3dLocalizer)

find_package(PCL 1.2 REQUIRED)
set(OpenCV_DIR /home/michal/OpenCV/Debug/)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#ADD HERE FURTHER SOURCE FILES
set(3D_SOURCE 3dLocalizer.cpp Caster.cpp Utils.cpp Quaternion.cpp TransformationMatrix.cpp Sphere.cpp)
set(TEST_SOURCE Test.cpp Caster.cpp Utils.cpp Quaternion.cpp TransformationMatrix.cpp Sphere.cpp)

add_executable (3dLocalizer_exe ${TEST_SOURCE})
target_link_libraries (3dLocalizer_exe ${PCL_LIBRARIES} ${OpenCV_LIBS})