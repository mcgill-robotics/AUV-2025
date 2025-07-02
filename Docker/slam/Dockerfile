# Use your custom base image
FROM mcgillrobotics/auv_2025:base

# Ensure UTF-8 locales
ENV LC_ALL=en_US.UTF-8 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en

# 1) Install RTAB-Map and all ROS dependencies
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-noetic-rtabmap-ros \
    ros-noetic-rtabmap \
    ros-noetic-fiducial-msgs \
    ros-noetic-grid-map-msgs ros-noetic-grid-map-core ros-noetic-grid-map-ros \
    ros-noetic-octomap-msgs ros-noetic-octomap \
    ros-noetic-apriltag-ros \
    python3-rosdep python3-rosinstall python3-colcon-common-extensions \
    git wget zstd \
    && rm -rf /var/lib/apt/lists/*

# 2) Update rosdep (base image already has rosdep initialized)
RUN rosdep update

# 3) Create a non-root 'zed' user for running the ZED SDK installer
RUN useradd -ms /bin/bash zed && \
    echo 'zed ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers && \
    mkdir -p /usr/local/zed && \
    chown zed:zed /usr/local/zed

# 4) Switch to 'zed' and install ZED SDK (installer refuses to run as root)
USER zed
WORKDIR /home/zed

RUN wget -O ZED_SDK.run \
    https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.0/ZED_SDK_Ubuntu20_cuda11.8_v4.0.3.zstd.run && \
    chmod +x ZED_SDK.run && \
    ./ZED_SDK.run --quiet --accept && \
    rm ZED_SDK.run

# 5) Back to root to clone & build the ROS wrappers
USER root
WORKDIR /AUV-2025/catkin_ws

# 6) Export ZED SDK paths so CMake can locate ZEDConfig.cmake
ENV ZED_SDK_ROOT=/usr/local/zed
ENV ZED_DIR=/usr/local/zed
ENV CMAKE_PREFIX_PATH=/usr/local/zed:$CMAKE_PREFIX_PATH
ENV LD_LIBRARY_PATH=/usr/local/zed/lib:$LD_LIBRARY_PATH

# 7) Clone ZED ROS wrapper and RTAB-Map integration, then build
RUN mkdir -p src && cd src && \
    git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git && \
    git clone --branch noetic-devel https://github.com/introlab/rtabmap_ros.git && \
    cd .. && \
    rosdep install --from-paths src --ignore-src -r -y && \
    /bin/bash -lc "source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release"

# 8) Entrypoint: source ROS and workspace
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
