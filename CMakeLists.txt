cmake_minimum_required(VERSION 2.8.3)
project(undistort_images)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_filters
  camera_info_manager
  image_transport
)

catkin_package()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES undistort_images
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)
add_executable(publishCameraInfo src/publishCameraInfo.cpp)
target_link_libraries(publishCameraInfo ${catkin_LIBRARIES})

add_executable(publishStereoCameraInfo src/publishStereoCameraInfo.cpp)
target_link_libraries(publishStereoCameraInfo ${catkin_LIBRARIES})
