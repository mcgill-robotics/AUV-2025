# Use the official ROS Noetic base image
FROM ros:noetic-ros-base

# Update package lists and install necessary tools
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    wget \
    python3-pip

# Initialize rosdep and update
RUN if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init; \
fi

# Update rosdep
RUN rosdep update

# Install catkin tools
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && wget http://packages.ros.org/ros.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y python3-catkin-tools

# Install ROS packages and dependencies
RUN apt-get install -y \
    ros-noetic-rosserial-arduino \
    ros-noetic-pid \
    ros-noetic-joy \
    ros-noetic-joy-teleop \
    ros-noetic-sbg-driver \
    ros-noetic-cv-bridge \
    ros-noetic-image-view \
    ros-noetic-rqt-gui \
    ros-noetic-ros-ign-gazebo \
    ros-noetic-ros-ign \
    ros-noetic-ros-ign-bridge

# Install Python dependencies
RUN pip3 install numpy-quaternion ultralytics

# Remove the CMake binary to avoid conflicts
RUN rm /usr/local/bin/cmake

# Set up the user environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
# Clean up
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Replace 'user' with the desired user name
# ENV HOME /home/$user_name
# USER $user_name

# Set the default command to start a shell
CMD ["/bin/bash"]
