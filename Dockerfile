# Use the official ROS Noetic base image
FROM ros:noetic-ros-base
ENV IGNITION_VERSION=fortress

# Install all needed deps and compile the mesa llvmpipe driver from source.
RUN sudo apt-get update \
    && sudo apt-get install -y software-properties-common \
    && sudo apt-get update \
    && sudo add-apt-repository ppa:kisak/kisak-mesa \
    && sudo apt-get update \
    && sudo apt-get -y upgrade

# Install catkin tools
RUN apt-get update && apt-get install -y wget \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && wget http://packages.ros.org/ros.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y python3-catkin-tools\
    && apt-get install -y python3-pip \
    && apt-get install -y \
    ros-noetic-rosserial-arduino \
    ros-noetic-pid \
    ros-noetic-joy \
    ros-noetic-joy-teleop \
    ros-noetic-sbg-driver \
    ros-noetic-cv-bridge \
    ros-noetic-image-view \
    ros-noetic-rqt-gui \
    ros-noetic-ros-ign-gazebo \
    ros-noetic-ros-ign \
    ros-noetic-ros-ign-bridge \
    ros-noetic-smach-ros \
    && pip3 install --no-cache-dir numpy-quaternion ultralytics \
    && rm /usr/local/bin/cmake  \
    && sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends ignition-fortress \
    && sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && sudo apt-get update \
    && sudo apt install -y ros-noetic-ros-ign \
    && sudo apt-get install -y ros-noetic-ros-ign-bridge \
    && sudo apt-get install -y ros-noetic-ros-ign-gazebo \
    && sudo apt-get install -y ros-noetic-gazebo-ros \
    && sudo apt-get install -y git \
    && pip3 install --no-cache-dir numpy==1.22.2 \
    && sudo apt-get autoremove -y  \
    && sudo apt-get clean && rm -rf /var/lib/apt/lists/* \
    && rm -rf /root/.cache/pip \