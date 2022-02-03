cmake_minimum_required(VERSION 2.8.3)
project(state_estimation)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  #std_msgs
  #geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
  #std_msgs
  #auv_msgs
  #angles
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(XimuReceiver imu/XimuReceiver.cpp)
target_link_libraries(XimuReceiver ${catkin_LIBRARIES})
add_dependencies(XimuReceiver ${catkin_EXPORTED_TARGETS})

add_executable(XimuPublisher imu/XimuPublisher.cpp)
target_link_libraries(XimuPublisher ${catkin_LIBRARIES})
add_dependencies(XimuPublisher ${catkin_EXPORTED_TARGETS})

###########
## Build ##
###########