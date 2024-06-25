FROM mcgillrobotics/auv_2024:sim

ENV DEBIAN_FRONTEND=noninteractive

RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin \
    && mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600 \
    && wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb \
    && dpkg -i cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb \
    && cp /var/cuda-repo-ubuntu2004-11-8-local/cuda-*-keyring.gpg /usr/share/keyrings/ \
    && apt-get update \
    && apt-get -y install cuda

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
    && sudo apt-get clean && rm -rf /var/lib/apt/lists/* \
    && rm -rf /root/.cache/pip

RUN apt-get update && apt-get install -y tmux vim iputils-ping iproute2 \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean