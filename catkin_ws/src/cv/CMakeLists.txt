cmake_minimum_required(VERSION 2.8.3)
project(cv)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  angles
  auv_msgs
  cmake_modules
  cv_bridge
  eigen_conversions
  geometry_msgs
  nav_msgs
  pointgrey_camera_driver
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions
  dynamic_reconfigure
)
add_action_files(
  DIRECTORY action
  FILES
  Call_ORB.action
)

find_package(Boost REQUIRED COMPONENTS)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CvTarget.msg
)

catkin_python_setup()

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  config/laneDetectorParams.cfg
)

catkin_package(CATKIN_DEPENDS
  actionlib_msgs
  angles
  auv_msgs
  boost
  cv_bridge
  darknet_ros
  eigen_conversions
  pointgrey_camera_driver
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions
  dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(lane_detector src/cv/shapeAndColorDetectors/lane_detector.cpp)
target_link_libraries(lane_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(torpedo_target src/cv/shapeAndColorDetectors/torpedo_targeter.cpp)
target_link_libraries(torpedo_target ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(bin_detector src/cv/shapeAndColorDetectors/bin_detector.cpp)
target_link_libraries(bin_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
