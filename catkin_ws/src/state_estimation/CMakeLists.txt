cmake_minimum_required(VERSION 3.0.2)
project(state_estimation)

find_package(catkin REQUIRED COMPONENTS
	rospy	
	std_msgs	
)

catkin_package( CATKIN_DEPENDS
	rospy
	std_msgs
)

