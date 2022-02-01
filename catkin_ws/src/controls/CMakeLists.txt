cmake_minimum_required(VERSION 3.0.2)
project(controls)

find_package(catkin REQUIRED COMPONENTS
	actionlib
	auv_msgs	
	pid
	rospy
	std_msgs
)

catkin_package(	CATKIN_DEPENDS 
	actionlib
	auv_msgs
	pid
	rospy
	std_msgs
)

