cmake_minimum_required(VERSION 3.0.2)
project(slam)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	sensor_msgs
	rtabmap_ros
	zed_wrapper
	rviz
)

catkin_package()

# Install launch files
install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install parameter files
install(DIRECTORY params/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
)

# Install RViz config files
install(DIRECTORY rviz/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)

