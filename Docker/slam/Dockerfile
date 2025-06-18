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

#install ros packages for rtabmap
RUN apt-get update && apt-get install -y \
    ros-noetic-rtabmap \
    ros-noetic-rtabmap-ros \
    python3-catkin-tools \
    python3-rosdep \
    git \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# 6) Initialize rosdep
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
    && rosdep init \
    && rosdep update --rosdistro $ROS_DISTRO

# 7) Create & populate overlay workspace
WORKDIR /workspace/catkin_ws
RUN mkdir -p src \
    && git clone --recursive --branch noetic-devel \
    https://github.com/introlab/rtabmap_ros.git \
    src/rtabmap_ros

# 8) Build the overlay
RUN /bin/bash -lc "\
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    catkin init && \
    rosdep install --from-paths src --ignore-src -y && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -- -j1 \
    "

# 9) Entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]