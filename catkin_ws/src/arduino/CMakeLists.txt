cmake_minimum_required(VERSION 2.8.3)
project(arduino)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  std_msgs
)

rosserial_generate_ros_lib(
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY thrusters
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(thrusters firmware ALL)
rosserial_add_client_target(thrusters firmware-upload)

rosserial_configure_client(
  DIRECTORY depth
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(depth firmware ALL)