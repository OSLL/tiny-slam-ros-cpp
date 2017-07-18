cmake_minimum_required(VERSION 2.8.3)

SET(CMAKE_CXX_FLAGS "-Wall -O3 -pthread")

add_compile_options(-std=c++11)

project(tiny_slam)

set (rosv "rosversion")
set (rosv_dist "-d")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf
  message_filters
)

include_directories(
	  include/${PROJECT_NAME}
	  ${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(tiny_slam src/tiny_slam/tiny_slam.cpp)

#if(CATKIN_ENABLE_TESTING)
#	find_package(GTest REQUIRED)
	include_directories(${GTEST_INCLUDE_DIRS})
  	  
	catkin_add_gtest(atest test/AutoExpandbleMap/AutoExpandble_map_utest.cpp)

	target_link_libraries(atest ${catkin_LIBRARIES} ${GTEST_LIBRARY})
#endif()

target_link_libraries(tiny_slam ${catkin_LIBRARIES})



