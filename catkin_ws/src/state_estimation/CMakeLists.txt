cmake_minimum_required(VERSION 2.8.3)
project(state_estimation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  auv_msgs
  angles
)

include_directories(${catkin_INCLUDE_DIRS})

###########
## Build ##
###########