FROM ubuntu:22.04

# 1. Install general build tools and Python packages
RUN apt-get update && apt-get install -y \
    build-essential cmake git python3-pip python3-flake8 \
    python3-pytest \
    wget curl gnupg2 lsb-release

# Install colcon via pip
RUN pip3 install -U colcon-common-extensions

# 2. Install ROS 1 Noetic from apt (needs focal-based packages)
RUN apt-get install -y \
    gnupg2 && \
    echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install -y ros-noetic-ros-base \
    && rm -rf /var/lib/apt/lists/*

# 3. Install ROS 2 Humble from source
RUN mkdir -p /ros2_humble && cd /ros2_humble \
    && wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos -O ros2.repos \
    && apt-get update && apt-get install -y \
         python3-vcstool \
         build-essential \
         libproject-dev \
         && vcs import src < ros2.repos \
    && rosdep update && rosdep install --from-paths src --ignore-src -y \
    && colcon build --symlink-install

# 4. Build ros1_bridge
RUN cd / \
    && git clone https://github.com/ros2/ros1_bridge.git -b humble src/ros1_bridge \
    && . /opt/ros/noetic/setup.bash \
    && . /ros2_humble/install/setup.bash \
    && colcon build --packages-select ros1_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release
