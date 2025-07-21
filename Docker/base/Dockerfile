# Use the official ROS Noetic base image
FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    LC_ALL=en_US.UTF-8 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en

RUN apt-get update \
    && apt-get install -y locales \
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
    && locale-gen \
    && apt-get install -y --no-install-recommends \
        wget \
        git \
        git-lfs \
        python3-catkin-tools \
        python3-pip \
        python-is-python3 \
        kmod \
        kbd \
        tmux \
        vim \
        iputils-ping \
        iproute2 \
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
        ros-noetic-foxglove-bridge \
        ros-noetic-xacro \
        ros-noetic-robot-state-publisher \
        ros-noetic-rtabmap \
        ros-noetic-rtabmap-ros \
        ros-noetic-nmea-msgs \
        ros-noetic-mavros-msgs \
        zstd \
    && curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt requirements.txt
RUN pip install --no-cache-dir -r requirements.txt \
    && rm -f /usr/local/bin/cmake \
    && rm -rf /root/.cache/pip \
    && rm requirements.txt

SHELL ["/bin/bash", "-c"]

RUN git clone --recursive https://github.com/mcgill-robotics/AUV-2025.git

RUN adduser jetson --disabled-password --gecos "" 
WORKDIR /root/
RUN wget https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.1/ZED_SDK_Tegra_L4T35.4_v4.1.4.zstd.run && \
    chmod +x ZED_SDK_Tegra_L4T35.4_v4.1.4.zstd.run
RUN ./ZED_SDK_Tegra_L4T35.4_v4.1.4.zstd.run -- silent && \
    chown -R jetson:jetson /usr/local/zed
USER root
WORKDIR /AUV-2025/catkin_ws

RUN source /opt/ros/noetic/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin build && \
    source devel/setup.bash

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && echo "source /AUV-2025/catkin_ws/devel/setup.bash" >> ~/.bashrc
