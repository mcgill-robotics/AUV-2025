cmake_minimum_required(VERSION 3.5)
project(sim)

# Find ament_cmake package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # For ROS2 C++ nodes
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sbg_driver REQUIRED)  # Ensure you have ROS2 support for this package

# Add any additional include directories
include_directories(include)

# TODO: Add executables and libraries here
# TODO: Replace with actual source files and targets
add_executable(example_node src/example_node.cpp)
ament_target_dependencies(example_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  sbg_driver
)

# Install targets
install(TARGETS
  example_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
