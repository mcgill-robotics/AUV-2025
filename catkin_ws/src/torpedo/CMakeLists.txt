cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arduino-cmake/cmake/ArduinoToolchain.cmake)
set(ARDUINO_USB_PORT /dev/ttyACM0)

# macro(crosscompile target toolchain_file)
# ExternalProject_Add(
# ${target}
# SOURCE_DIR ${CMAKE_SOURCE_DIR}
# PREFIX ${PREFIX_DIR}/${target}
# BINARY_DIR ${BUILD_DIR}/${target}
# INSTALL_DIR ${INSTALL_DIR}/${target}
# DEPENDS vtk-compile-tools
# ${BUILD_ALWAYS_STRING}
# CMAKE_ARGS
# -DCMAKE_CROSSCOMPILING:BOOL=ON
# -DCMAKE_TOOLCHAIN_FILE:FILEPATH=${toolchain_file}
# -DCMAKE_MAKE_PROGRAM:FILEPATH=${CMAKE_MAKE_PROGRAM}
# )
										      
project(torpedo)

# find_package(catkin REQUIRED)
# catkin_package()

###########
## Build ##
###########
set(FIRMWARE_TARGET ${PROJECT_NAME}_firmware)
generate_arduino_firmware(${FIRMWARE_TARGET}
	BOARD uno
	SKETCH ${CMAKE_SOURCE_DIR}/torpedo
	PORT ${ARDUINO_USB_PORT}
	)

set(UPLOAD_TARGET ${FIRMWARE_TARGET}-upload)
add_custom_target(COMPILE_AND_UPLOAD ALL
	DEPENDS ${UPLOAD_TARGET}
	)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/torpedo_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_torpedo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
