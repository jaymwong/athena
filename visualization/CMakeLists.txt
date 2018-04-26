cmake_minimum_required(VERSION 2.8.3)
project(athena_visualization)

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
	LIBRARIES athena_visualization
	CATKIN_DEPENDS roscpp message_generation
	DEPENDS EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
	include
)

add_library(athena_visualization
	src/utils.cpp
)

add_dependencies(athena_visualization
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(athena_visualization
  ${catkin_LIBRARIES}
)
