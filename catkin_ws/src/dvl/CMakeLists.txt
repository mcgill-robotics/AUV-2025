cmake_minimum_required(VERSION 3.0.2)
project(dvl)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  geometry_msgs
  message_generation
  rospy
  std_msgs
  auv_msgs
)

catkin_python_setup()

add_message_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  rospy
  std_msgs
  auv_msgs
)

install(PROGRAMS
  scripts/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
