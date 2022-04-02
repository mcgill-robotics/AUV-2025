cmake_minimum_required(VERSION 3.0.2)
project(controls)

set(MSG_DEP_SET auv_msgs geometry_msgs std_msgs)

find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
	actionlib
	pid
	rospy
	tf
)

catkin_package(	CATKIN_DEPENDS 
	${MSG_DEP_SET}
	actionlib
	pid
	rospy
	tf
)

