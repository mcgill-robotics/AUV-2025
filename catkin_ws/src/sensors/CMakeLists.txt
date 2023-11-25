cmake_minimum_required(VERSION 3.0.2)
project(sensors)

set(MSG_DEP_SET std_msgs auv_msgs)

find_package(catkin REQUIRED COMPONENTS
  ${MSG_DEP_SET}
  rospy
  roscpp
  rosserial_arduino
  rosserial_client
  geometry_msgs
  sbg_driver
)

catkin_package( CATKIN_DEPENDS
  ${MSG_DEP_SET}
  roscpp
  rospy
  rosserial_arduino
  rosserial_client
  geometry_msgs
  sbg_driver
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY embedded
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

# Isn't this wrong? Shouldn't it be 
# rosserial_add_client_target(
#   embedded
#   ALL
#   ALL
# )
rosserial_add_client_target(
  embedded
  sensors
  ALL
)

rosserial_add_client_target(
  embedded
  depth_sensor-upload
)

install(PROGRAMS
  scripts/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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


