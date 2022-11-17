cmake_minimum_required(VERSION 3.0.2)

project(imu)

include_directories(${ROS_LIB_DIR})

set(CMAKE_CXX_STANDARD 11)

generate_arduino_firmware(${PROJECT_NAME}
	SKETCH imu
	BOARD uno


	#BOTH LINES COPIED FROM DEPTH SENSOR NEEDS TO BE LOOKED INTO
	SRCS ${ROS_LIB_DIR}/time.cpp
	PORT dev/ttyACM0
)
