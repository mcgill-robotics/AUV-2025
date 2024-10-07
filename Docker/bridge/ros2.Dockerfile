# Building Humble Container
FROM osrf/ros:humble-desktop

# Setup Base environments again 
RUN apt-get update && apt-get install -y locales\
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    locale-gen

ENV LC_ALL en_US.UTF-8 
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en     

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*

# Due to ROS 2 being ran on multiple ws, might have to config in future
WORKDIR /auv
COPY /colcon_ws ./colcon_ws

# Copies built files from catkin to new workspace
# COPY --from=0 ../catkin_ws ../auv/catkin_ws 
# COPY --from=0 ../opt/ros ../opt/ros

# Performs Check on Environment Variables for Humble (UNIX-systems)
RUN printenv | grep -i ROS

# Current Build Only Runs package "bridge_pubsub"
RUN cd colcon_ws\
    && colcon build --packages-select bridge_pubsub\
    %% cd .. 

# Building Bridge environment variables continued from ROS 2)

# Sources config files on terminal bootup by adding to .bashrc
RUN echo "source ../opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && echo "source ../opt/ros/humble/setup.bash" >> ~/.bashrc 

RUN echo "source /install/setup.bash" >> ~/.bashrc 

# CMD ["ros2", "launch", "bridge_pubsub", "subscriber_function.py"]