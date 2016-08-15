cmake_minimum_required(VERSION 2.8)
SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -O3")
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(tiny_slam src/tiny_slam/tiny_slam.cpp)
