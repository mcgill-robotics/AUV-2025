cmake_minimum_required(VERSION 3.0.2)
project(calibration)

# set of packages whose message types are used
set(MSG_DEP_SET auv_msgs) 

find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
	rospy
	controls
    	propulsion
)

catkin_package(CATKIN_DEPENDS 
	${MSG_DEP_SET}
	rospy
)
