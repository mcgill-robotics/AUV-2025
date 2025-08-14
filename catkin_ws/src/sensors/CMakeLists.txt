cmake_minimum_required(VERSION 3.0.2)
project(sensors)

set(MSG_DEP_SET std_msgs sensor_msgs auv_msgs)

find_package(catkin REQUIRED COMPONENTS
  ${MSG_DEP_SET}
  rospy
  roscpp
  geometry_msgs
  sbg_driver
  rosserial_arduino
  rosserial_client
  tf2
  message_filters
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS
    ${MSG_DEP_SET}
    roscpp
    rospy
    geometry_msgs
    sbg_driver
    rosserial_arduino
    rosserial_client
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

## Compile depth_republish.cpp into a ROS node
add_executable(depth_republisher src/depth_republish.cpp)
add_executable(waterlinked_driver src/waterlinked_driver.cpp)
add_executable(movella_republisher src/movella_republisher.cpp)
target_link_libraries(depth_republisher ${catkin_LIBRARIES})
target_link_libraries(movella_republisher ${catkin_LIBRARIES})   # <<< ADD THIS
target_link_libraries(waterlinked_driver ${catkin_LIBRARIES})


## Install the compiled node
install(TARGETS depth_republisher waterlinked_driver movella_republisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install your Python and shell scripts
install(PROGRAMS
  src/start_zed_bridge.sh
  scripts/${PROJECT_NAME}
  src/imu_covariance.py
  src/dvl_covariance.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
