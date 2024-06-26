#!/bin/bash
source /opt/ros/noetic/setup.bash
mkdir zed
cd zed
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
git clone https://github.com/stereolabs/zed-ros-wrapper.git
git clone https://github.com/stereolabs/zed-ros-interfaces.git
cd ..
catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install;
# echo "source /zed/catkin_ws/install/setup.bash" >> ~/.bashrc