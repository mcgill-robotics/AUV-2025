cmake_minimum_required(VERSION 3.0.2)
project(auv_msgs)

find_package(catkin REQUIRED COMPONENTS
  # roscpp
  # rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(FILES
  ThrusterCommands.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS 
  std_msgs 
  message_runtime
)