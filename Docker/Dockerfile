# Use the official ROS Noetic base image
FROM ros:noetic-ros-base
ENV IGNITION_VERSION=fortress

# Install all needed deps and compile the mesa llvmpipe driver from source.
RUN sudo apt-get update \
    && apt-get install -y software-properties-common \
    && apt-get update \
    && add-apt-repository ppa:kisak/kisak-mesa \
    && apt-get update \
    && apt-get -y upgrade \
    && apt-get update && apt-get install -y wget \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && wget http://packages.ros.org/ros.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y python3-catkin-tools python3-pip \
    && apt-get install -y \
    ros-noetic-rosserial-arduino \
    ros-noetic-pid \
    ros-noetic-joy \
    ros-noetic-joy-teleop \
    ros-noetic-sbg-driver \
    ros-noetic-cv-bridge \
    ros-noetic-image-view \
    ros-noetic-rqt-gui \
    ros-noetic-smach-ros 

RUN pip3 install --no-cache-dir numpy-quaternion ultralytics \
    && rm -f /usr/local/bin/cmake  \
    && sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y --no-install-recommends ignition-fortress \
    && apt-get update \
    && apt install -y ros-noetic-ros-ign \
    ros-noetic-ros-ign-bridge \
    ros-noetic-ros-ign-gazebo \
    ros-noetic-gazebo-ros \
    git \
    && pip3 install --no-cache-dir numpy==1.22.2 \
    && sudo apt-get autoremove -y  \
    && sudo apt-get clean && rm -rf /var/lib/apt/lists/* \
    && rm -rf /root/.cache/pip \
    && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && echo "source /root/AUV-2024/catkin_ws/devel/setup.bash" >> ~/.bashrc