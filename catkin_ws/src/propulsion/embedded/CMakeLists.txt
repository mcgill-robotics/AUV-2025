cmake_minimum_required(VERSION 3.0.2)
project(propulsion)

include_directories(${ROS_LIB_DIR})

# generated message file (ie. ThrusterCommand.h) use C++ 11 features
# avoids error: nullptr was not declared in this scope
# a better solution may be warranted 
set (CMAKE_CXX_STANDARD 11)

# Define the path to the Arduino CLI executable
set(ARDUINO_CLI_PATH "/usr/local/bin/arduino-cli")

# Custom target to compile the embedded code
add_custom_target(compile_teensy
    COMMAND ${ARDUINO_CLI_PATH} compile --library ${CMAKE_SOURCE_DIR}/../../../build/propulsion/ros_lib --fqbn teensy:avr:teensy40 ${CMAKE_SOURCE_DIR}/thrusters
    COMMENT "Compiling code for Teensy"
)

# Custom target to upload the compiled code to Teensy
add_custom_target(upload_teensy
    DEPENDS compile_teensy
    COMMAND ${ARDUINO_CLI_PATH} upload -p /dev/ttyACM0 --fqbn teensy:avr:teensy40 ${CMAKE_SOURCE_DIR}/thrusters
    COMMENT "Uploading code to Teensy"
)

# Dummy target which allows ros_lib to be generated during build
add_custom_target(thrusters
    COMMENT "No build action specified for target thrusters."
)
