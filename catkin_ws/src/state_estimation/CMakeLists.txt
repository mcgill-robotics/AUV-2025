cmake_minimum_required(VERSION 3.0.2)
project(state_estimation)

set(MSG_DEP_SET geometry_msgs std_msgs)

find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
	depth_sensor
	rospy
	tf
)

catkin_package( CATKIN_DEPENDS
	${MSG_DEP_SET}
	depth_sensor
	rospy
	tf
)
