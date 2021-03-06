cmake_minimum_required(VERSION 3.0.2)
project(controls)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  auv_msgs
  #roscpp
  rospy
)

catkin_package(
  CATKIN_DEPENDS
  auv_msgs
  geometry_msgs
  rospy
  std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
