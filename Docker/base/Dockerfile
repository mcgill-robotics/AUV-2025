# Use the official ROS Noetic base image
FROM ros:noetic-ros-base

# Setup environment
ENV DEBIAN_FRONTEND=noninteractive \
    LC_ALL=en_US.UTF-8 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en

ARG TARGETPLATFORM

# Install apt packages
RUN apt-get update -qq \
    && apt-get install -y locales \
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
    && locale-gen \
    && apt-get install -y --no-install-recommends -qq \
        wget \
        tar \
        perl \
        build-essential \
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
        zstd \
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
    && curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash \
    && rm -rf /var/lib/apt/lists/*

# Install python packages
COPY requirements.txt requirements.txt
RUN pip install -qq --no-cache-dir -r requirements.txt \
    && rm -f /usr/local/bin/cmake \
    && rm -rf /root/.cache/pip \
    && rm requirements.txt

# Set shell to bash
SHELL ["/bin/bash", "-c"]

RUN echo "Target platform: $TARGETPLATFORM" && \
    if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
        echo "Installing x86_64-specific dependencies"; \
    elif [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
        echo "Installing ARM64-specific dependencies"; \
    fi

# Install Cuda 11.4.0/11.8.0
RUN if [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
        echo "Installing CUDA 11.4 for ARM64"; \
        wget -q https://developer.download.nvidia.com/compute/cuda/11.4.0/local_installers/cuda_11.4.0_470.42.01_linux_sbsa.run && \
        chmod +x cuda_11.4.0_470.42.01_linux_sbsa.run && \
        ./cuda_11.4.0_470.42.01_linux_sbsa.run --silent --toolkit && \
        rm cuda_11.4.0_470.42.01_linux_sbsa.run; \
    else \
        echo "Installing CUDA 11.8 for x86_64"; \
        wget -q https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run && \
        chmod +x cuda_11.8.0_520.61.05_linux.run && \
        ./cuda_11.8.0_520.61.05_linux.run --extract=/tmp/cuda && \
        dpkg -i /tmp/cuda/cuda-toolkit-11-8_11.8.89-1_amd64.deb && \
        rm -rf /tmp/cuda cuda_11.8.0_520.61.05_linux.run
    fi

# Set Cuda environment variables
# ENV PATH=/usr/local/cuda-11.4/bin:$PATH
# ENV LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:$LD_LIBRARY_PATH

# Install ZED SDK
RUN adduser jetson --disabled-password --gecos "" 
WORKDIR /root/
RUN if [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
        echo "Installing ZED SDK for ARM64 Tegra"; \
        wget -q https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.1/ZED_SDK_Tegra_L4T35.4_v4.1.4.zstd.run && \
        chmod +x ZED_SDK_Tegra_L4T35.4_v4.1.4.zstd.run && \
        ./ZED_SDK_Tegra_L4T35.4_v4.1.4.zstd.run -- silent && \
        chown -R jetson:jetson /usr/local/zed; \
        rm ZED_SDK_Tegra_L4T35.4_v4.1.4.zstd.run; \
    else \
        echo "Installing ZED SDK for x86_64 Ubuntu20 CUDA11.8"; \
        wget -q https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.1/ZED_SDK_Ubuntu20_cuda11.8_v4.1.4.zstd.run && \
        chmod +x ZED_SDK_Ubuntu20_cuda11.8_v4.1.4.zstd.run && \
        ./ZED_SDK_Ubuntu20_cuda11.8_v4.1.4.zstd.run -- silent && \
        rm ZED_SDK_Ubuntu20_cuda11.8_v4.1.4.zstd.run; \
    fi
USER root
WORKDIR /

# Add source to bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && echo "source /AUV-2025/catkin_ws/devel/setup.bash" >> ~/.bashrc
