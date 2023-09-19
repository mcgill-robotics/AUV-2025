# Use the official ROS Noetic base image
FROM ros:noetic-ros-base

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
    && rm /usr/local/bin/cmake \
    && apt-get autoremove -y \
    && apt-get clean && rm -rf /var/lib/apt/lists/* \
    && rm -rf /root/.cache/pip