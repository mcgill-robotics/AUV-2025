#!/bin/bash
set -e
# source ROS and workspace overlays
source "/opt/ros/noetic/setup.bash"
source "/AUV-2025/catkin_ws/devel/setup.bash"
exec "$@"
