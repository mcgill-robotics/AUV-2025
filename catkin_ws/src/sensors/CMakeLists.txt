cmake_minimum_required(VERSION 3.0.2)
project(sensors)

set(MSG_DEP_SET std_msgs sensor_msgs auv_msgs)

find_package(catkin REQUIRED COMPONENTS
  ${MSG_DEP_SET}
  roscpp
  geometry_msgs
  sbg_driver
  rosserial_arduino
  rosserial_client
  tf2
  tf2_ros
  message_filters 
  serial
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 3.3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    ${MSG_DEP_SET}
    roscpp
    geometry_msgs
    sbg_driver
    rosserial_arduino
    rosserial_client
    tf2
    tf2_ros
    message_filters
    serial
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

## Compile executables
add_executable(depth_republisher src/depth_republish.cpp)
add_executable(movella_republish src/movella_republish.cpp)
add_executable(zed_pose_republisher src/zed_pose_republisher.cpp)
add_executable(waterlinked_driver src/dvl_covariance.cpp)

## Link libraries
target_link_libraries(depth_republisher 
  ${catkin_LIBRARIES}
)

target_link_libraries(movella_republish 
  ${catkin_LIBRARIES}
)

target_link_libraries(zed_pose_republisher
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

target_link_libraries(waterlinked_driver 
  ${catkin_LIBRARIES}
  ${serial_LIBRARIES}
)

## Install the compiled nodes
install(TARGETS depth_republisher movella_republish zed_pose_republisher waterlinked_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Python and shell scripts
install(PROGRAMS
  src/sensors_status.py
  src/tether_status.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)