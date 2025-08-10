FROM dustynv/pytorch:2.1-r35.4.1

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

#key expired? -> fetch the current key and store it in /usr/share/keyrings
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros1-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros1-archive-keyring.gpg] \
    http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros1.list

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic

RUN apt update \
    && apt install -y openssh-server xauth \
    && mkdir /var/run/sshd \
    && mkdir /root/.ssh \
    && chmod 700 /root/.ssh \
    && echo "Port 222" >> /etc/ssh/sshd_config \
    && echo "PermitRootLogin yes" >> /etc/ssh/sshd_config \
    && echo 'root:jetson' | chpasswd

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

ENV IGNITION_VERSION=fortress

# Install all needed deps and compile the mesa llvmpipe driver from source.
RUN apt-get update && apt-get install -y wget \
    && wget http://packages.ros.org/ros.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y python3-catkin-tools python3-pip tmux vim

RUN apt-get install -y ros-noetic-rosserial-arduino \
    ros-noetic-pid \
    ros-noetic-joy \
    ros-noetic-joy-teleop \
    ros-noetic-sbg-driver \
    ros-noetic-cv-bridge \
    ros-noetic-image-view \
    ros-noetic-rqt-gui \
    ros-noetic-rviz \
    ros-noetic-smach-ros \
    ros-noetic-usb-cam \
    ros-noetic-robot-localization \
    ros-noetic-foxglove-bridge \
    ros-noetic-xacro \
    ros-noetic-robot-state-publisher \
    ros-noetic-rtabmap \
    ros-noetic-rtabmap-ros \
    ros-noetic-nmea-msgs \
    ros-noetic-mavros-msgs

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libjpeg-dev \
    zlib1g-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

COPY requirements.txt requirements.txt
RUN pip install --no-cache-dir -r requirements.txt \
    && pip install --no-deps --no-cache-dir ultralytics==8.0.207 \
    && rm -f /usr/local/bin/cmake && rm -rf /root/.cache/pip && rm requirements.txt

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && echo "source /AUV-2025/catkin_ws/devel/setup.bash" >> ~/.bashrc \
    && echo "source /AUV-2025/scripts/set_env.bash" >> ~/.bashrc \
    && echo "source /AUV-2025/scripts/jetson/docker_aliases.bash" >> ~/.bashrc

# RUN git clone --branch release/0.16 --recursive --depth=1 https://github.com/pytorch/vision torchvision && \
#     cd torchvision && \
#     git checkout release/0.16 && \
#     python3 setup.py bdist_wheel && \
#     cp dist/torchvision*.whl /opt && \
#     pip3 install --no-cache-dir --verbose /opt/torchvision*.whl && \
#     cd ../ && \
#     rm -rf torchvision \
#     && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
#     && echo "source /AUV-2025/catkin_ws/devel/setup.bash" >> ~/.bashrc \
#     && echo "source /AUV-2025/scripts/set_env.bash" >> ~/.bashrc \
#     && echo "source /AUV-2025/scripts/jetson/docker_aliases.bash" >> ~/.bashrc

RUN apt-get update && apt-get install -y python-is-python3 kmod kbd iputils-ping iproute2 \
    && pip3 install numpy-quaternion keyboard \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

SHELL ["/bin/bash", "-c"]

ENV BUILD_VERSION=0.16.1
RUN curl -o ~/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl && \
   python3 -m pip install --no-cache ~/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl && \
   sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev

RUN apt-get update && apt-get install -y docker.io && pip install scikit-learn && pip install dill
RUN git clone --recursive https://github.com/mcgill-robotics/AUV-2025.git
RUN git clone --branch v0.16.1 --recursive https://github.com/pytorch/vision.git /torchvision

RUN apt update \
    && apt install -y zstd

# Create New User for Jetson, tag ensures dockerfile keeps working regardless of any changes in base img
RUN adduser jetson --disabled-password --gecos "" 

# Download & Install ZED SDK
WORKDIR /root/
RUN wget https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.1/ZED_SDK_Tegra_L4T35.4_v4.1.4.zstd.run && \
    chmod +x ZED_SDK_Tegra_L4T35.4_v4.1.4.zstd.run

# Run ZED Installer as Root (Safer) Then Change Ownership
RUN ./ZED_SDK_Tegra_L4T35.4_v4.1.4.zstd.run -- silent && \
    chown -R jetson:jetson /usr/local/zed

COPY SN33688213.conf /usr/local/zed/settings/

USER root
WORKDIR /AUV-2025/catkin_ws

RUN source /opt/ros/noetic/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin build && \
    source devel/setup.bash

# Create a dynamic variable to fix TLS stack memory issue with imports and activate torchvision
RUN echo "export LD_PRELOAD=$(sudo find / -iname libgomp.so.1 | head -1)" >> ~/.bashrc
RUN echo "[ -d /torchvision ] && cd /torchvision && python3 setup.py install --user && cd /AUV-2025/catkin_ws && rm -rf /torchvision" >> ~/.bashrc

SHELL ["/bin/bash", "-c"]
RUN echo "tmux" >> ~/.bashrc
