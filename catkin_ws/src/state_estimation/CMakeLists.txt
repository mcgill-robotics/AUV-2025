cmake_minimum_required(VERSION 3.0.2)
project(state_estimation)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy	
	rosserial_arduino
	rosserial_client
	std_msgs	
)

catkin_package( CATKIN_DEPENDS
	roscpp
	rospy
	rosserial_arduino
	rosserial_client
	std_msgs
)

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
