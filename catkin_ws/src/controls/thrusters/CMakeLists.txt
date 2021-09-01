cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(firmware
  SKETCH thrusters.ino
  SRCS ${ROS_LIB_DIR}/time.cpp
  HDRS thrusters.h ros.h
  BOARD uno
  PORT /dev/thrusters
)
