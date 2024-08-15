cmake_minimum_required(VERSION 3.8)
project(state_estimation)

# Default to C++17 if not set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set dependencies
set(MSG_DEP_SET geometry_msgs std_msgs auv_msgs nav_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(${MSG_DEP_SET} REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(sbg_driver REQUIRED)

# Include directories
include_directories(
  include
)

# Add executable
add_executable(odom_republisher src/odom_republisher.cpp)

# Link libraries with ament_target_dependencies
ament_target_dependencies(odom_republisher
  rclcpp
  sensor_msgs
  ${MSG_DEP_SET}
  tf2
  tf2_geometry_msgs
  tf2_ros
  sbg_driver
)

# Install targets
install(
  TARGETS odom_republisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(
  DIRECTORY include/
  DESTINATION include/
)

# Export dependencies
ament_export_dependencies(${MSG_DEP_SET} rclcpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros sbg_driver)
ament_package()
