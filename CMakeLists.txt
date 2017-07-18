cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -O3 -pthread")
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

catkin_package()

add_executable(tiny_slam src/tiny_slam/tiny_slam.cpp)

execute_process(COMMAND ${rosv} ${rosv_dist}
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
	RESULT_VARIABLE ros_result
	OUTPUT_VARIABLE ros_version)

if(${ros_version} STREQUAL "jade\n" OR (${ros_version} STREQUAL "indigo\n")  )
  	target_link_libraries(tiny_slam ${catkin_LIBRARIES})
else()
 	if(CATKIN_ENABLE_TESTING)
 		find_package(GTest)
  		include_directories(${catkin_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})
  		catkin_add_gtest(atest test/AutoExpandbleMap/AutoExpandble_map_utest.cpp)
	  	target_link_libraries(tiny_slam ${catkin_LIBRARIES} ${GTEST_LIBRARY})
 	endif()
endif()




