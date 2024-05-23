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
    && pip3 install scikit-learn

ARG TARGETPLATFORM

# Download ZED installer depending on architecture
RUN if [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
        wget -O zed_sdk_install_ubuntu.run https://download.stereolabs.com/zedsdk/4.1/l4t35.4/jetsons; \
    elif [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
        wget -O zed_sdk_install_ubuntu.run https://download.stereolabs.com/zedsdk/4.1/cu118/ubuntu20; \
    fi

# Install ZED camera deps
RUN apt-get update \
    && apt-get install -y zstd \
    && apt-get install -y ros-noetic-usb-cam \
    && pip3 install bitstring \
    && chmod +x zed_sdk_install_ubuntu.run \
    && ./zed_sdk_install_ubuntu.run -- silent \
    && rm zed_sdk_install_ubuntu.run
    
RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y --no-install-recommends ignition-fortress \
    && apt-get update \
    && apt install -y ros-noetic-ros-ign \
    ros-noetic-ros-ign-bridge \
    ros-noetic-ros-ign-gazebo \
    ros-noetic-gazebo-ros

RUN rm -f /usr/local/bin/cmake  \
    && sudo apt-get autoremove -y  \
    && sudo apt-get clean && rm -rf /var/lib/apt/lists/* \
    && rm -rf /root/.cache/pip \
    && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && echo "source /root/AUV-2024/catkin_ws/devel/setup.bash" >> ~/.bashrc