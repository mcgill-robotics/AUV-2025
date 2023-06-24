cmake_minimum_required(VERSION 3.0.2)
project(hydrophones)

set(MSG_DEP_SET)

find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
	rospy
)

catkin_package( CATKIN_DEPENDS
	${MSG_DEP_SET}
	rospy
)

