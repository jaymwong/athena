cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_conversions)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
	COMPONENTS
	message_generation
	pcl_ros
	pcl_conversions
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)


#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES pointcloud_conversions
	CATKIN_DEPENDS roscpp message_generation pcl_ros pcl_conversions
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

add_library(pointcloud_conversions src/utils.cpp src/conversions.cpp)
add_dependencies(pointcloud_conversions ${catkin_EXPORTED_TARGETS} )
target_link_libraries(pointcloud_conversions ${catkin_LIBRARIES} ${OpenCV_LIBS} )
