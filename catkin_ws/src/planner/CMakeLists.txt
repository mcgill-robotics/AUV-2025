
cmake_minimum_required(VERSION 3.0.2)
project(planner)

find_package(catkin REQUIRED COMPONENTS
	actionlib
	auv_msgs	
	rospy
	std_msgs
)

catkin_package(	CATKIN_DEPENDS 
	actionlib
	auv_msgs
	rospy
	smach
	std_msgs
)

