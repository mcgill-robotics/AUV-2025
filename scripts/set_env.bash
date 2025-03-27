#!/usr/bin/env bash

# Only set environment variables if ubuntu.local is reachable
if ping -c 1 ubuntu.local &> /dev/null; then
  echo "ubuntu.local is reachable."
  export ROS_IP="ubuntu.local"
  export ROS_MASTER_URI="http://ubuntu.local:11311"
else
  echo "ubuntu.local not reachable - not setting environment variables."
fi
