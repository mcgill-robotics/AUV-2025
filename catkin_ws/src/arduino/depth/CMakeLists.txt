cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(firmware
  SKETCH depth.ino
  SRCS ms5803_i2c.cpp ${ROS_LIB_DIR}/time.cpp
  HDRS depth.h ms5803_i2c.h ros.h
  BOARD leonardo
  PORT /dev/ttyACM0
)
