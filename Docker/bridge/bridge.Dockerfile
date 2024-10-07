# Use the official ROS Noetic base image
FROM ros:noetic-ros-base AS ros1

RUN apt-get update && apt-get install -y locales\
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    locale-gen

ENV LC_ALL en_US.UTF-8 
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en     

# Install all needed deps and compile the mesa llvmpipe driver from source.
RUN apt-get update && apt-get install -y wget \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && wget http://packages.ros.org/ros.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y python3-catkin-tools python3-pip python-is-python3 kmod kbd tmux vim iputils-ping iproute2 \
    && apt-get install -y --no-install-recommends \
    ros-noetic-rosserial-arduino \
    ros-noetic-pid \
    ros-noetic-joy \
    ros-noetic-joy-teleop \
    ros-noetic-sbg-driver \
    ros-noetic-cv-bridge \
    ros-noetic-image-view \
    ros-noetic-rqt-gui \
    ros-noetic-rviz \
    ros-noetic-smach-ros \
    ros-noetic-robot-localization \ 
    git-lfs \
    git \
    && curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash 

RUN pip3 install --no-cache-dir numpy-quaternion keyboard \
    && pip install --no-cache-dir --upgrade numpy

COPY /catkin_ws ./catkin_ws

RUN rm -f /usr/local/bin/cmake  \
    && sudo apt-get autoremove -y  \
    && sudo apt-get clean && rm -rf /var/lib/apt/lists/* \
    && rm -rf /root/.cache/pip 

RUN /bin/bash -c '. ./opt/ros/noetic/setup.bash; cd catkin_ws; catkin_make' 

# Building Humble Container
FROM osrf/ros:humble-desktop

# Setup Base environments again 
RUN apt-get update && apt-get install -y locales\
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    locale-gen

ENV LC_ALL en_US.UTF-8 
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en     

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*

# Performs Check on Environment Variables for Humble (UNIX-systems)
RUN printenv | grep -i ROS

COPY --from=ros1 /opt/ros ./opt/ros
COPY --from=ros1 /catkin_ws ./catkin_ws

# Creates /AUV-2025 and prepares to build three workspaces inside
RUN mkdir AUV-2025\
    && cd AUV-2025

# Builds ROS 1 Noetic using catkin_make, sources /setup.bash into ROS 1 Bridge
COPY /catkin_ws ./catkin_ws

# Builds ROS 2 Humble using colcon build --package-select, sources /setup.bash into ROS 1 Bridge
COPY /colcon_ws ./colcon_ws
RUN cd colcon_ws\
    && colcon build --packages-select bridge_pubsub\
    %% cd .. 

COPY /ros-humble-ros1-bridge ./ros_bridge_ws

# TODO: Add sourcing steps so that ros_1_bridge can build without any setup