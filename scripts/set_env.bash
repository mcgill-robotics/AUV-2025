#!/usr/bin/env bash

# Only set environment variables if the Jetson IP is reachable
if ping -c 1 192.168.0.105 &> /dev/null; then
  echo "Jetson IP reachable."
  # Attempt to extract the IP from common interfaces (wlp or eth)
  IP=$(ip addr show | grep -E 'inet ' | grep -E 'wlp|eth' | awk '{print $2}' | cut -d/ -f1 | head -n 1)
  if [ -n "$IP" ]; then
      export ROS_IP=$IP
  else
      export ROS_IP=192.168.0.105
  fi

  # Set the master node to the Jetson
  export ROS_MASTER_URI=http://192.168.0.105:11311
else
    echo "Jetson IP not reachable - not setting environment variables."
fi
