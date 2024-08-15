cmake_minimum_required(VERSION 3.0.2)
project(controls)

set(MSG_DEP_SET auv_msgs geometry_msgs std_msgs)

find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
	actionlib
    joy
    joy_teleop
	pid
	rospy
	tf
	tf2
	tf2_geometry_msgs
	tf2_ros
)

catkin_package(	CATKIN_DEPENDS 
	${MSG_DEP_SET}
	actionlib
    joy
    joy_teleop
	pid
	rospy
	tf
	tf2
	tf2_geometry_msgs
	tf2_ros
)

