cmake_minimum_required(VERSION 3.0.2)
project(dvl)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  auv_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  rospy
  std_msgs
  auv_msgs
)

install(PROGRAMS
  scripts/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
