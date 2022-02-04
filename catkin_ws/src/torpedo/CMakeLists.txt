cmake_minimum_required(VERSION 3.0.2)
project(torpedo)

# set of packages whose message types are used
set(MSG_DEP_SET std_msgs) # no dependencies

find_package(catkin REQUIRED COMPONENTS
		${MSG_DEP_SET}
		roscpp
		rosserial_arduino
		rosserial_client
)

catkin_package(CATKIN_DEPENDS 
		${MSG_DEP_SET}
		roscpp
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
		torpedo_firmware
		ALL
)

rosserial_add_client_target(
		embedded
		torpedo_firmware-upload
)
