cmake_minimum_required(VERSION 2.8.3)
project(tools)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  controls
)

# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## None for now...

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
  sensor_msgs
  std_msgs
  controls
)

###########
## Build ##
###########

## None for now...

#############
## Install ##
#############

## None for now...

#############
## Testing ##
#############

## None for now...
