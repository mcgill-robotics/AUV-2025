FROM mcgillrobotics/auv_2024:sim

# Download ZED installer depending on architecture
RUN if [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
        wget -O zed_sdk_install_ubuntu.run https://download.stereolabs.com/zedsdk/4.1/l4t35.4/jetsons; \
    elif [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
        wget -O zed_sdk_install_ubuntu.run https://download.stereolabs.com/zedsdk/4.1/cu118/ubuntu20; \
    fi

# Install ZED camera deps
RUN apt-get update \
    && apt-get install -y zstd \
    && apt-get install -y ros-noetic-usb-cam \
    && pip3 install bitstring \
    && chmod +x zed_sdk_install_ubuntu.run \
    && ./zed_sdk_install_ubuntu.run -- silent \
    && rm zed_sdk_install_ubuntu.run

RUN mkdir zed && cd zed && mkdir catkin_ws && cd catkin_ws && mkdir src \
    && cd src && git clone https://github.com/stereolabs/zed-ros-wrapper.git \
    && cd .. && catkin_make -DCMAKE_BUILD_TYPE=Release install \
    && echo "source /zed/catkin_ws/install/setup.bash" >> ~/.bashrc