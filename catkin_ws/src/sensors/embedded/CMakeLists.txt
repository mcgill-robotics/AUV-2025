cmake_minimum_required(VERSION 3.0.2)

project(sensors)

# Depth sensor

include_directories(${ROS_LIB_DIR})

# generated message file (ie. ThrusterCommand.h) use C++ 11 features
# avoids error: nullptr was not declared in this scope
# a better solution may be warranted 
set (CMAKE_CXX_STANDARD 11)

generate_arduino_firmware(${PROJECT_NAME}
  SKETCH depth_sensor
  BOARD mega2560
  
  # this is needed for node_handle
  # avoids error: undefined reference to `ros::normalizeSecNSec(unsigned long&, unsigned long&)'
  # feels like a bit of a hack, look into above documentation for neater solution
  SRCS ${ROS_LIB_DIR}/time.cpp ${CMAKE_SOURCE_DIR}/depth_sensor/MS5803.cpp
  HDRS ${CMAKE_SOURCE_DIR}/depth_sensor/MS5803.h
  PORT /dev/power
)

# IMU 

#[[generate_arduino_firmware(${PROJECT_NAME}
  SKETCH imu
  BOARD mega2560

  #BOTH LINES COPIED FROM DEPTH SENSOR NEEDS TO BE LOOKED INTO
  SRCS ${ROS_LIB_DIR}/time.cpp ${CMAKE_SOURCE_DIR}/imu/Quaternion.cpp ${CMAKE_SOURCE_DIR}/imu/XimuReceiver.cpp 
  HDRS ${CMAKE_SOURCE_DIR}/imu/Quaternion.h ${CMAKE_SOURCE_DIR}/imu/XimuReceiver.h
  PORT /dev/imu
)]]

# DISPLAY

# generate_arduino_firmware(${PROJECT_NAME}
#   SKETCH display
#   BOARD mega2560

#   SRCS ${ROS_LIB_DIR}/time.cpp
#   PORT /dev/display
# )