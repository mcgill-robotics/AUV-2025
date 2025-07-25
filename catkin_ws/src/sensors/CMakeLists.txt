cmake_minimum_required(VERSION 3.0.2)
project(sensors)

set(MSG_DEP_SET std_msgs sensor_msgs auv_msgs)

find_package(catkin REQUIRED COMPONENTS
  ${MSG_DEP_SET}
  rospy
  roscpp
  geometry_msgs
  sbg_driver
  rosserial_arduino
  rosserial_client
)

catkin_package( CATKIN_DEPENDS
  ${MSG_DEP_SET}
  roscpp
  rospy
  geometry_msgs
  sbg_driver
  rosserial_arduino
  rosserial_client
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

install(PROGRAMS
  src/start_zed_bridge.sh
  scripts/${PROJECT_NAME}
  src/imu_covariance.py
  src/dvl_covariance.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)