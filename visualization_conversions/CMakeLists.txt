cmake_minimum_required(VERSION 2.8.3)
project(visualization_conversions)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
	COMPONENTS
		geometry_msgs
  	tf_conversions
		message_generation
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES visualization_conversions
	CATKIN_DEPENDS roscpp message_generation
	DEPENDS EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
	include
)

add_library(visualization_conversions
	src/utils.cpp
)

add_dependencies(visualization_conversions
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(visualization_conversions
  ${catkin_LIBRARIES}
)
