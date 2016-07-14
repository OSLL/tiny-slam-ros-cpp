cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -O3")
project(tiny_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  message_filters
  sensor_msgs
  rosbag_storage
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(tiny_slam src/tiny_slam/tiny_slam.cpp)

target_link_libraries(tiny_slam ${catkin_LIBRARIES})

