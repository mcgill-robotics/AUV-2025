cmake_minimum_required(VERSION 3.0.2)
project(state_estimation)

set(MSG_DEP_SET geometry_msgs std_msgs auv_msgs)

find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
	depth_sensor
	imu
	rospy
    tf
	tf2
	tf2_geometry_msgs
	tf2_ros
    sbg_driver
)

catkin_package( CATKIN_DEPENDS
	${MSG_DEP_SET}
	depth_sensor
	imu
	rospy
    tf
	tf2
	tf2_geometry_msgs
	tf2_ros
    sbg_driver
)
