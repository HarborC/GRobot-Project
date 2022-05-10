cmake_minimum_required(VERSION 3.0)
project(sensor_publisher)

#---------------------------------------------------------------------------------------
# find package and the dependecy
#---------------------------------------------------------------------------------------
find_package(Boost 1.54 REQUIRED COMPONENTS
	system
	thread
	chrono
	)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	sensor_msgs
	std_msgs
	message_generation
	rosbag
	nav_msgs
	geometry_msgs
	livox_ros_driver
	cv_bridge
	image_transport
	)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects als    o need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also n    eed
catkin_package(CATKIN_DEPENDS
	roscpp rospy std_msgs message_runtime
	)

#---------------------------------------------------------------------------------------
# Set default build to release
#---------------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

#---------------------------------------------------------------------------------------
# Compiler config
#---------------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(
	${catkin_INCLUDE_DIRS}
  	${PCL_INCLUDE_DIRS}
	${OPENCV_INCLUDE_DIRS}
  	include)

add_executable(recordor_and_convertor_node src/recordor_and_convertor.cpp)
target_link_libraries(recordor_and_convertor_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OPENCV_LIBRARIES})