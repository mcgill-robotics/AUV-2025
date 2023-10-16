cmake_minimum_required(VERSION 3.0.2)
project(sensors)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  rosserial_arduino
  rosserial_client
  std_msgs  
)

catkin_package( CATKIN_DEPENDS
  rospy
  roscpp
  rosserial_arduino
  rosserial_client
  std_msgs
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY embedded
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(
  embedded
  sensors
  ALL
)
rosserial_add_client_target(
  embedded
  depth_sensor-upload
)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  auv_msgs
)

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

set(MSG_DEP_SET std_msgs auv_msgs)

find_package(catkin REQUIRED COMPONENTS
  ${MSG_DEP_SET}
  roscpp
  rospy
  rosserial_arduino
  rosserial_client
    sbg_driver
)

catkin_package( CATKIN_DEPENDS
  ${MSG_DEP_SET}
  roscpp
  rospy
  rosserial_arduino
  rosserial_client
    sbg_driver
)
#[[
rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)


rosserial_configure_client(
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
  DIRECTORY embedded
)

rosserial_add_client_target(
  embedded
  imu
  ALL
)

rosserial_add_client_target(
  embedded
  imu-upload
)]]


