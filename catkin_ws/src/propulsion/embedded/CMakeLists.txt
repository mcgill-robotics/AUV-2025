cmake_minimum_required(VERSION 2.8.3)

project(thrusters)

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(${PROJECT_NAME}
  SKETCH thrusters
  BOARD uno
  PORT dev/ttyACM0 #TODO - udev rules map to dev/thrusters
)
