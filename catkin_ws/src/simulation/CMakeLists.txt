cmake_minimum_required(VERSION 3.0.2)
project(simulation)

# TODO - set enc fortress
set(MSG_DEP_SET std_msgs)

find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
	rospy
	ros_ign_bridge
	ros_ign_gazebo

)

catkin_package(	CATKIN_DEPENDS 
	${MSG_DEP_SET}
	rospy
	ros_ign_bridge
	ros_ign_gazebo
)

