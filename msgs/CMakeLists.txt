cmake_minimum_required(VERSION 2.8.3)
project(athena_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
	COMPONENTS
	message_generation
  std_msgs
)

find_package(cmake_modules REQUIRED)

add_message_files(DIRECTORY msg)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp message_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
