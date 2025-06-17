# Use the official ROS Noetic base image
FROM mcgillrobotics/auv_2025:base
RUN sudo wget http://packages.osrfoundation.org/gazebo.key && apt-key add gazebo.key && apt-get update \
    && apt-get install -y --no-install-recommends ignition-fortress \
    && apt-get update \
    && apt install -y ros-noetic-ros-ign \
    ros-noetic-ros-ign-bridge \
    ros-noetic-ros-ign-gazebo \
    ros-noetic-gazebo-ros

RUN apt-get -y update \
    && apt-get install -y software-properties-common \
    && apt-get update \
    && add-apt-repository ppa:kisak/kisak-mesa \
    && apt-get update \
    && apt-get -y upgrade \
    && apt-get install -y libglapi-mesa xvfb
# && apt-get install -y mesa-utils libglu1-mesa-dev freeglut3-dev mesa-common-dev \
# libglew-dev libglfw3-dev libglm-dev libao-dev libmpg123-dev
