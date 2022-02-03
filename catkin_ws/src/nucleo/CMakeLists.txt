cmake_minimum_required(VERSION 2.8.3)
project(nucleo)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Signals.msg
   Debug_Signals.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

catkin_package(
    CATKIN_DEPENDS message_runtime
)