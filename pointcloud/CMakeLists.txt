cmake_minimum_required(VERSION 2.8.3)
project(athena_pointcloud)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
	COMPONENTS
	message_generation
	pcl_ros
	pcl_conversions
	athena_transform
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(DIRECTORY msg
  FILES
  CloudGeometry.msg
  CloudGeometries.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
   
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES athena_pointcloud
	CATKIN_DEPENDS roscpp message_generation pcl_ros pcl_conversions athena_transform
	DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(athena_pointcloud
	src/utils.cpp
	src/conversions.cpp
	src/planar.cpp
	src/bounding_geometry.cpp
)
add_dependencies(athena_pointcloud ${catkin_EXPORTED_TARGETS} )
target_link_libraries(athena_pointcloud ${catkin_LIBRARIES} ${OpenCV_LIBS} )
