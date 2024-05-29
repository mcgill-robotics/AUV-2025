FROM mcgillrobotics/auv_2024:sim

ENV DEBIAN_FRONTEND=noninteractive

RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.0-1_all.deb \
    && dpkg -i cuda-keyring_1.0-1_all.deb \
    && apt-get update \
    && apt-get install -y --no-install-recommends cuda

# Install ZED camera deps
RUN wget -O zed_sdk_install_ubuntu.run https://download.stereolabs.com/zedsdk/4.1/cu118/ubuntu20 \
    && apt-get update \
    && apt-get install -y zstd ros-noetic-usb-cam \
    && pip3 install bitstring \
    && chmod +x zed_sdk_install_ubuntu.run \
    && ./zed_sdk_install_ubuntu.run -- silent skip_cuda \
    && rm zed_sdk_install_ubuntu.run

COPY build_zed.bash .

RUN chmod +x build_zed.bash \
    && /bin/bash -c './build_zed.bash' \
    && rm build_zed.bash \
    && echo "source /zed/catkin_ws/install/setup.bash" >> ~/.bashrc \
    && sudo apt-get clean && rm -rf /var/lib/apt/lists/* \
    && rm -rf /root/.cache/pip \