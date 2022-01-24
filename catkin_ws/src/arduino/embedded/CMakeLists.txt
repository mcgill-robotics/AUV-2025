
cmake_minimum_required(VERSION 3.5.0)
project(firmware)

# generated message file (ie. ThrusterCommands.h) use C++ 11 features
# avoids error: ‘nullptr’ was not declared in this scope
# a better solution may be warranted - looking into arduino toolchain
set (CMAKE_CXX_STANDARD 11)

include_directories(${ROS_LIB_DIR})

add_definitions(-DUSB_CON)


# See https://github.com/queezythegreat/arduino-cmake for documentation
generate_arduino_firmware(${PROJECT_NAME}
  SKETCH thrusters

  # this is needed for node_handle
  # avoids error: undefined reference to `ros::normalizeSecNSec(unsigned long&, unsigned long&)'
  # feels like a bit of a hack, look into above documentation for neater solution
  SRCS ${ROS_LIB_DIR}/time.cpp

  BOARD uno
  PORT /dev/ttyACM0
)
