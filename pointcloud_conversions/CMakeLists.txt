cmake_minimum_required(VERSION 2.8.3)
project(transform_conversions)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
	COMPONENTS
		message_generation
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES transform_conversions
	CATKIN_DEPENDS roscpp message_generation
	DEPENDS EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
	include
)
