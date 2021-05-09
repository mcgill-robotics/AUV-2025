cmake_minimum_required(VERSION 2.8.3)
project(blinky)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

add_message_files(FILES
  RGB.msg
  RGBArray.msg
)

add_service_files(FILES
  UpdateComputerBatteryLights.srv
  UpdateMotorBatteryLights.srv
  UpdatePlannerLights.srv
  WarningLights.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  rospy
  std_msgs
)

catkin_install_python(PROGRAMS
  scripts/blinky_node
  scripts/blinky_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
