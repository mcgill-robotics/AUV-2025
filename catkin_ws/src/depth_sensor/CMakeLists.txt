cmake_minimum_required(VERSION 3.0.2)
project(depth_sensor)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rosserial_arduino
	rosserial_client
	std_msgs	
)

catkin_package( CATKIN_DEPENDS
	roscpp
	rosserial_arduino
	rosserial_client
	std_msgs
)

#[[for the pool test, the firmware is already uploaded
rosserial_generate_ros_lib(
	PACKAGE rosserial_arduino
	SCRIPT make_libraries.py
)

rosserial_configure_client(
	DIRECTORY embedded
	TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(
	embedded
	depth_sensor
	ALL
)
rosserial_add_client_target(
	embedded
	depth_sensor-upload
)
]]
