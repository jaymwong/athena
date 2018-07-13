cmake_minimum_required(VERSION 2.8.3)
project(athena)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
	COMPONENTS
		athena_transform
  	athena_pointcloud
		athena_visualization
		athena_parsers
)

catkin_package(
	INCLUDE_DIRS include
#	LIBRARIES athena
	CATKIN_DEPENDS roscpp message_generation athena_transform athena_pointcloud athena_visualization athena_parsers
	DEPENDS EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
	include
)
