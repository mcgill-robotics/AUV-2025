cmake_minimum_required(VERSION 2.8.3)
project(controls)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib
  actionlib_msgs
  genmsg
  geometry_msgs
  rospy
  std_msgs
  tf
  teleop_twist_joy
)

catkin_python_setup()

add_message_files(
  FILES
  ThrusterCommands.msg
)
generate_messages(DEPENDENCIES
#     geometry_msgs
     std_msgs
#     actionlib_msgs
 )

catkin_package(
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  genmsg
  geometry_msgs
  rospy
  std_msgs
  tf
  teleop_twist_joy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# catkin_install_python(PROGRAMS none
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

