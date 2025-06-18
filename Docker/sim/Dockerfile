FROM mcgillrobotics/auv_2025:base

# Fix GPG key errors from foxglove installation
RUN sudo rm /etc/apt/sources.list.d/ros-latest.list && sudo rm /etc/apt/sources.list.d/ros1-latest.list
RUN sudo apt update
RUN sudo apt install -y curl gnupg2 lsb-release
RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/ros1-latest.list
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN sudo apt update
RUN sudo apt install -y software-properties-common
RUN sudo add-apt-repository universe
RUN sudo apt update
#

RUN sudo apt-get update && sudo apt-get install -y ros-noetic-foxglove-bridge

RUN pip3 install --no-cache-dir numpy-quaternion ultralytics \
    && pip install --no-cache-dir --upgrade numpy \
    && pip3 install scikit-learn

RUN rm -f /usr/local/bin/cmake  \
    && sudo apt-get autoremove -y  \
    && sudo apt-get clean && rm -rf /var/lib/apt/lists/* \
    && rm -rf /root/.cache/pip