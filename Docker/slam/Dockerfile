FROM mcgillrobotics/auv_2025:base

ENV ROS_DISTRO=noetic
ENV ZED_DIR=/usr/local/zed 

SHELL ["/bin/bash", "-lc"]

# remove outdated ROS1 source list
RUN rm -f /etc/apt/sources.list.d/ros1-latest.list || true

# add ROS apt key
RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
RUN apt-get update && apt-get install -y \
    ca-certificates \
    apt-transport-https \
    && rm -rf /var/lib/apt/lists/*

# 1) install curl and gnupg to fetch ROS key
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release \
    && rm -rf /var/lib/apt/lists/*

# 2) fetch and install the current ROS apt key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | apt-key add -

# 3) recreate the ROS apt source, signed by that key
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/trusted.gpg.d/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros-latest.list

# 4) move the key into trusted.gpg.d
RUN apt-key export F42ED6FBAB17C654 \
    | gpg --dearmor \
    > /etc/apt/trusted.gpg.d/ros-archive-keyring.gpg

# 5) install ROS packages: RTAB-Map, RViz, robot_localization, and tools
RUN apt-get update && apt-get install -y \
    ros-noetic-rtabmap \
    ros-noetic-rtabmap-ros \
    ros-noetic-rviz \
    ros-noetic-robot-localization \
    python3-catkin-tools \
    python3-rosdep \
    git \
    build-essential \
    cmake \
    libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

RUN wget -qO /tmp/ZED_SDK_Ubuntu.deb \
    https://download.stereolabs.com/zedsdk/4.8/Ubuntu20

RUN apt-get update && \
    apt-get install -y /tmp/ZED_SDK_Ubuntu.deb && \
    rm /tmp/ZED_SDK_Ubuntu.deb


# (optional) confirm where the CMake file landed
RUN find /usr/local/zed -name ZEDConfig.cmake

# ensure CMake will search it
ENV CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/zed/cmake



# right after the .deb install
RUN dpkg -L zedsdk | grep -i ZEDConfig.cmake || \
    find /usr/local -name ZEDConfig.cmake

ENV CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/zed/cmake

# 6) Initialize rosdep
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
    && rosdep init \
    && rosdep update --rosdistro $ROS_DISTRO

# 7) Create & populate overlay workspace
WORKDIR /workspace/catkin_ws
RUN mkdir -p src \
    # clone RTAB-Map ROS wrapper
    && git clone --recursive --branch noetic-devel \
    https://github.com/introlab/rtabmap_ros.git \
    src/rtabmap_ros \
    # clone ZED ROS wrapper from GitHub
    && git clone --recursive \
    https://github.com/stereolabs/zed-ros-wrapper.git \
    src/zed_wrapper \
    && git clone --recursive --branch noetic-devel \
    https://github.com/ros-perception/image_transport_plugins.git \
    src/image_transport_plugins  

RUN apt-get update && apt-get install -y \
    libjpeg-turbo8-dev

# 8) Build the overlay (including RTAB-Map and ZED wrapper)
RUN /bin/bash -lc "\
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    catkin init && \
    rosdep install --from-paths src --ignore-src -y && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -- -j1 \
    "

# set display for macOS XQuartz
ENV DISPLAY=host.docker.internal:0

# 9) Entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
