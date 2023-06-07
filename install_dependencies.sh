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
#for vision visualization
pip install scipy
#DVL dependencies
rosdep update
#IntelRealSense driver
sudo apt-get install -y ros-noetic-realsense2-camera
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules
