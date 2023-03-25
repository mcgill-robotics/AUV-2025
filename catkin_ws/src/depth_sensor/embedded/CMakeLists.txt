cmake_minimum_required(VERSION 3.0.2)

project(depth_sensor)

include_directories(${ROS_LIB_DIR})

# generated message file (ie. ThrusterCommand.h) use C++ 11 features
# avoids error: nullptr was not declared in this scope
# a better solution may be warranted 
set (CMAKE_CXX_STANDARD 11)

generate_arduino_firmware(${PROJECT_NAME}
	SKETCH depth_sensor
	BOARD uno
	
	# this is needed for node_handle
	# avoids error: undefined reference to `ros::normalizeSecNSec(unsigned long&, unsigned long&)'
	# feels like a bit of a hack, look into above documentation for neater solution
	SRCS ${ROS_LIB_DIR}/time.cpp ${CMAKE_SOURCE_DIR}/depth_sensor/MS5803.cpp
	HDRS ${CMAKE_SOURCE_DIR}/depth_sensor/MS5803.h
	PORT dev/ttyACM0 #TODO - udev rules map to dev/thrusters
)
