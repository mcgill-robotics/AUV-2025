#NOTE: to avoid errors, make sure you install Ubuntu with the "Minimal installation" option selected and without third-party software enabled.

# INSTALL ROS NOETIC
echo "source /opt/ros/noetic/setup.bash" >> /home/$user_name/.bashrc
source /home/$user_name/.bashrc
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update

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
sudo apt-get install -y ros-noetic-ros-ign-gazebo
#python dependencies
sudo apt install -y python3-pip
sudo apt install -y python-is-python3
pip3 install numpy-quaternion
pip3 install ultralytics

rosdep update
rm /usr/local/bin/cmake


# sudo apt-get update
# sudo apt-get install git
# sudo apt-get install -y lsb-release wget gnupg
# sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# sudo apt-get update
# sudo apt-get install -y ignition-fortress
# rosdep install -r --from-paths src -i -y --rosdistro noetic

# sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# sudo apt-get update
sudo apt install -y ros-noetic-ros-ign
sudo apt install -y ros-noetic-ros-ign-bridge
# git clone https://github.com/mcgill-robotics/auv-ros-ign-bridge catkin_ws/src/sim/auv-ros-ign-bridge
