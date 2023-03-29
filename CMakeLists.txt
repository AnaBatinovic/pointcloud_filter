cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  dynamic_reconfigure
  roslib
  cv_bridge
  image_transport
)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

add_compile_options(-std=c++11)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ransac_pointcloud2
#  CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories( include 
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS} )

## Declare a C++ executable
add_executable(pointcloud_filter
  src/main.cpp
  src/pc_pub_sub.cpp
  src/depth_image_pointcloud_filter.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
	pointcloud_filter
  	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	${OpenCV_LIBS}
)

add_definitions(${PCL_DEFINITIONS})