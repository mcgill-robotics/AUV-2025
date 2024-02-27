cmake_minimum_required(VERSION 3.0.2)
project(state_estimation)

set(MSG_DEP_SET geometry_msgs std_msgs auv_msgs)

find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
  sensors
  roscpp REQUIRED
	rospy
  tf
	tf2
	tf2_geometry_msgs
	tf2_ros
  sbg_driver
)

catkin_package( CATKIN_DEPENDS
	${MSG_DEP_SET}
  sensors
  roscpp
	rospy
  tf
	tf2
	tf2_geometry_msgs
	tf2_ros
  sbg_driver
	INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)


add_library(sensors_lib
  src/sensors.cpp
)

add_executable(${PROJECT_NAME}
  src/state_aggregator.cpp
)

target_link_libraries(
  ${PROJECT_NAME} 
  sensors_lib 
  ${roscpp_LIBRARIES} 
  ${tf2_ros_LIBRARIES}
  ${tf2_LIBRARIES}
)



install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)