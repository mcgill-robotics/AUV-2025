cmake_minimum_required(VERSION 3.0.2)
project(bringup)

find_package(colcon REQUIRED COMPONENTS
	controls	
	planner
	propulsion
	state_estimation
)
colcon_package(	COLCON_DEPENDS 
	controls	
	planner
	propulsion
	state_estimation
)

