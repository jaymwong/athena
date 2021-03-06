cmake_minimum_required(VERSION 2.8.3)
project(athena_transform)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
	COMPONENTS
		geometry_msgs
  	tf_conversions
		eigen_conversions
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_python_setup()


catkin_package(
	INCLUDE_DIRS include
	LIBRARIES athena_transform
	CATKIN_DEPENDS roscpp
	DEPENDS EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
	include
)

add_library(athena_transform
	src/conversions.cpp
)

add_dependencies(athena_transform
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(athena_transform
  ${catkin_LIBRARIES}
)
