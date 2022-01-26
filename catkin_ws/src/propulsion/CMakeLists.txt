cmake_minimum_required(VERSION 2.8.3)
project(propulsion)

# set of packages whose message types are used
set(MSG_DEP_SET propulsion_msgs) # no dependencies

find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
	message_generation
	roscpp
	rospy
	rosserial_arduino
	rosserial_client
)

catkin_package(CATKIN_DEPENDS 
	${MSG_DEP_SET}
	message_runtime
	roscpp
	rospy
	rosserial_arduino
	rosserial_client
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
	thrusters
	ALL
)
rosserial_add_client_target(
	embedded
	thrusters-upload
)
