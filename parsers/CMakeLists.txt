cmake_minimum_required(VERSION 2.8.3)
project(athena_parsers)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES athena_parsers
	CATKIN_DEPENDS roscpp
	DEPENDS EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
	include
)

add_library(athena_parsers src/parser_utils.cpp )
add_dependencies(athena_parsers ${catkin_EXPORTED_TARGETS} )
target_link_libraries(athena_parsers ${catkin_LIBRARIES} )
