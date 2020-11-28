cmake_minimum_required(VERSION 3.5.0)
project(arduino)

find_package(catkin REQUIRED COMPONENTS
    rosserial_arduino
    rosserial_client
)

catkin_package()

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY thrusters
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(thrusters firmware ALL)
rosserial_add_client_target(thrusters firmware-upload)