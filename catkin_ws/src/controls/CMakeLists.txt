cmake_minimum_required(VERSION 3.0.2)
project(controls)

find_package(catkin REQUIRED COMPONENTS
	pid
	std_msgs
	actionlib
	rospy
	auv_msgs	
)

# catkin_python_setup()

catkin_package(
	CATKIN_DEPENDS auv_msgs rospy actionlib std_msgs pid
)

