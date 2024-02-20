# Use the official ROS Noetic base image
FROM ros:noetic-ros-base

# Install all needed deps and compile the mesa llvmpipe driver from source.
RUN apt-get update && apt-get install -y wget \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && wget http://packages.ros.org/ros.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y python3-catkin-tools python3-pip python-is-python3 \
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
    git-lfs \
    git \
    && curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash 


RUN pip3 install --no-cache-dir numpy-quaternion ultralytics \
    && pip install --no-cache-dir --upgrade numpy \
    && pip3 install scikit-learn \
    && rm -f /usr/local/bin/cmake  \
    && sudo apt-get autoremove -y  \
    && sudo apt-get clean && rm -rf /var/lib/apt/lists/* \
    && rm -rf /root/.cache/pip \
    && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && echo "source /root/AUV-2024/catkin_ws/devel/setup.bash" >> ~/.bashrc