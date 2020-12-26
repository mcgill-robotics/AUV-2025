cmake_minimum_required(VERSION 3.0.2)
project(controls)

find_package(catkin REQUIRED COMPONENTS
  geometery_msgs
  auv_msgs
  python-numpy
  python3-scipy
  roscpp
  rospy
)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
