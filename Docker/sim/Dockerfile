FROM mcgillrobotics/auv_2025:base

ENV ROS_DISTRO=noetic
SHELL ["/bin/bash", "-lc"]
RUN rm -f /etc/apt/sources.list.d/ros1-latest.list || true

RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

#1)install curl and gnupg to fetch the new ROS key
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release \
    && rm -rf /var/lib/apt/lists/*

#2)fetch and install the current ROS apt key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | apt-key add - 

#3. recreate the ROS apt source, signed by that key
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/trusted.gpg.d/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros-latest.list

#4)move the key into the trusted.gpg.d directory
RUN apt-key export F42ED6FBAB17C654 \
    | gpg --dearmor \
    > /etc/apt/trusted.gpg.d/ros-archive-keyring.gpg

#5) Initialize rosdep
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
    && rosdep init \
    && rosdep update --rosdistro $ROS_DISTRO

RUN sudo apt-get update && sudo apt-get install -y ros-noetic-foxglove-bridge

RUN pip3 install --no-cache-dir numpy-quaternion ultralytics \
    && pip install --no-cache-dir --upgrade numpy \
    && pip3 install scikit-learn

RUN rm -f /usr/local/bin/cmake  \
    && sudo apt-get autoremove -y  \
    && sudo apt-get clean && rm -rf /var/lib/apt/lists/* \
    && rm -rf /root/.cache/pip