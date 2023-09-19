# Use the official ROS Noetic base image
FROM ros:noetic-ros-base

# Set the user name (update with your desired user name)
# ARG user_name=user

# Update package lists and install necessary tools
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    wget \
    python3-pip

# # Initialize rosdep and update
# RUN if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
#     rosdep init; \
# fi

# Update rosdep
# RUN rosdep update

# Install catkin tools
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && wget http://packages.ros.org/ros.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y python3-catkin-tools

# Install ROS packages and dependencies
RUN apt-get install -y 
RUN apt-get install ros-noetic-rosserial-arduino 
RUN apt-get install ros-noetic-pid
RUN apt-get install ros-noetic-joy 
RUN apt-get install ros-noetic-joy-teleop 
RUN apt-get install ros-noetic-sbg-driver
RUN apt-get install ros-noetic-cv-bridge
RUN apt-get install ros-noetic-image-view
RUN apt-get install ros-noetic-rqt-gui
RUN apt-get install ros-noetic-ros-ign-gazebo
RUN apt-get install ros-noetic-ros-ign
RUN apt-get install ros-noetic-ros-ign-bridge
RUN apt-get install ros-noetic-smach-ros

# Install Python dependencies
RUN pip3 install numpy-quaternion ultralytics

# RUN rosdep update

# Remove the CMake binary to avoid conflicts
RUN rm /usr/local/bin/cmake

# Set up the user environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
# Clean up
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get upgrade -y

# Replace 'user' with the desired user name
# ENV HOME /home/$user_name

# RUN sudo adduser $user_name
# RUN sudo adduser $user_name sudo

# USER $user_name

# Set the default command to start a shell
CMD ["bash"]
# CMD ["apt-get", "update", "&&", "apt-get", "install", "-y", "&&", "bin/bash"]