cmake_minimum_required(VERSION 3.0.2)
project(quali_gate)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   GateInfo.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
 )


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)