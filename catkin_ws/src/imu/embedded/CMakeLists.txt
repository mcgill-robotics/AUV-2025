cmake_minimum_required(VERSION 3.0.2)

project(imu)

include_directories(${ROS_LIB_DIR})

set(CMAKE_CXX_STANDARD 11)

#[[generate_arduino_firmware(${PROJECT_NAME}
	SKETCH imu
	BOARD mega2560

	#BOTH LINES COPIED FROM DEPTH SENSOR NEEDS TO BE LOOKED INTO
	SRCS ${ROS_LIB_DIR}/time.cpp ${CMAKE_SOURCE_DIR}/imu/Quaternion.cpp ${CMAKE_SOURCE_DIR}/imu/XimuReceiver.cpp 
	HDRS ${CMAKE_SOURCE_DIR}/imu/Quaternion.h ${CMAKE_SOURCE_DIR}/imu/XimuReceiver.h
	PORT /dev/ttyACM1
)]]
