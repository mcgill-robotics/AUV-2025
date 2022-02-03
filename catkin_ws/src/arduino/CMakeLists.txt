cmake_minimum_required(VERSION 3.5.0)
project(arduino)

find_package(catkin REQUIRED COMPONENTS
  auv_msgs
  rosserial_arduino
  rosserial_client
)

catkin_package(CATKIN_DEPENDS
  auv_msgs
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY embedded
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(embedded firmware ALL)
rosserial_add_client_target(embedded firmware-upload)

# # the rest of this has to do with applying patches, it mey not be necessary in the
# # future as fixes are currently awaiting release
# # patching Arduino.cmake requires sudo so it should be done manually

# # workaround to https://github.com/ros-drivers/rosserial/issues/518
# # msg.h includes code that refers to c++ standard library, however this is not
# # standard for arduino (avr). This patches portions of the code to use the C
# # version of libraries/functions that are part of the core library (avr-libc).
# # A dependency heirarchy is created as follows:
# # ros_lib <- APPLY_ROS_LIB_PATCH <- thursters <- thrusters_firmware, thrusters_firmware-upload
# add_custom_target(${PROJECT_NAME}_APPLY_ROS_LIB_PATCH

#   # the execution of an external script seems like a bit of a hack
#   # need to make sure the (existing) file is patched prior to building firmware
#   COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/patch/safe_patch.bash  ${CMAKE_CURRENT_BINARY_DIR}/ros_lib/ros/msg.h ${CMAKE_CURRENT_SOURCE_DIR}/patch/msg.h.patch
#   COMMENT "applying patch to ros_lib/ros/msg.h"
# ) 

# add_dependencies(${PROJECT_NAME}_APPLY_ROS_LIB_PATCH
#   ${PROJECT_NAME}_ros_lib
# )

# # rosserial_configure_client creates thrusters directory in /build to put make files (target: arduino_thrusters)
# # for firmware (subproject) targets (firmware, firmware-upload). These are dependents of arduino_thrusters
# # so prior to that the patch should be applied to the products of arduino_ros_lib
# add_dependencies(${PROJECT_NAME}_thrusters_firmware
#   ${PROJECT_NAME}_APPLY_ROS_LIB_PATCH
# )
