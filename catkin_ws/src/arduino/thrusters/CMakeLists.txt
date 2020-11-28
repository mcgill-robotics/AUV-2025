
cmake_minimum_required(VERSION 3.5.0)

include_directories(${ROS_LIB_DIR})

add_definitions(-DUSB_CON)

generate_arduino_firmware(firmware
  SKETCH thrusters.ino
  SRCS ${ROS_LIB_DIR}/time.cpp
  # HDRS thrusters.h ros.h
  BOARD uno
  PORT /dev/ttyACM0
)