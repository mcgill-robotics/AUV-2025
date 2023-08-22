#NOTE: to avoid errors, make sure you install Ubuntu with the "Minimal installation" option selected and without third-party software enabled.


# INSTALL ROS NOETIC
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
rm ./ros_install_noetic.sh

# INSTALL CATKIN TOOLS
sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y python3-catkin-tools

# PACKAGE DEPENDENCIES
sudo apt-get install -y ros-noetic-rosserial-arduino
sudo apt-get install -y ros-noetic-pid
sudo apt-get install -y ros-noetic-joy
sudo apt-get install -y ros-noetic-joy-teleop
sudo apt-get install -y ros-noetic-sbg-driver
sudo apt-get install -y ros-noetic-cv-bridge
sudo apt-get install -y ros-noetic-image-view
sudo apt-get install -y ros-noetic-rqt-gui
sudo apt-get install -y ros-noetic-rqt-gui-image-view
#python dependencies
sudo apt install -y python3-pip
pip3 install numpy-quaternion
pip3 install ultralytics
#DVL dependencies
rosdep update
#IntelRealSense driver
sudo apt-get install -y ros-noetic-realsense2-camera
sudo cp catkin_ws/src/vision/config/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules


#IF YOU GET THIS ERROR:
#   CMake Error: CMake was unable to find a build program corresponding to "Unix Makefiles".  CMAKE_MAKE_PROGRAM is not set.  You probably need to select a different build tool.
#   make[2]: *** [CMakeFiles/depth_sensor_embedded.dir/build.make:70: CMakeFiles/depth_sensor_embedded] Error 1
#   make[1]: *** [CMakeFiles/Makefile2:1378: CMakeFiles/depth_sensor_embedded.dir/all] Error 2
#   make: *** [Makefile:146: all] Error 2

# THEN RUN:
# which cmake
# cmake --version -> version 1
# cd /usr/bin/
# ./cmake --version -> version 2


# IF ONE OF THESE CATKIN VERSIONS IS NOT 3.16.3, THEN DELETE THAT VERSION
# TO DO SO:
# version 1: navigate to the folder that was returned by `which cmake`, do `rm cmake`
# version 2: navigate to /usr/bin, do `rm cmake`
# open new terminal 
# FINALLY: run `catkin clean` and build the packages again


## INSTALL CUDA IF POSSIBLE:
##     https://developer.nvidia.com/cuda-11-8-0-download-archive?target_os=Linux&target_arch=aarch64-jetson&Compilation=Native&Distribution=Ubuntu&target_version=20.04&target_type=deb_network