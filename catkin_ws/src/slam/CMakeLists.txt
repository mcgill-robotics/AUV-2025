# CMakeLists.txt
cmake_minimum_required(VERSION 3.0.2)
project(slam)

## Compile as C++11, supported in ROS Noetic
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  tf
  roslib
  roslaunch
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    nav_msgs
    sensor_msgs
    tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install launch files, parameter files, and RViz configs
install(
  DIRECTORY launch params rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
