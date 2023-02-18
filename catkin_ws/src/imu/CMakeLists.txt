cmake_minimum_required(VERSION 3.0.2)
project(imu)

set(MSG_DEP_SET std_msgs auv_msgs)

find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
	roscpp
	rospy
	rosserial_arduino
	rosserial_client
    sbg_driver
)

catkin_package( CATKIN_DEPENDS
	${MSG_DEP_SET}
	roscpp
	rospy
	rosserial_arduino
	rosserial_client
    sbg_driver
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
	imu
	ALL
)

rosserial_add_client_target(
	embedded
	imu-upload
)
