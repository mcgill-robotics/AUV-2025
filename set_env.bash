#!/usr/bin/env bash

# set ROS_IP to the IP adress of this host machine
IP=$(ip addr show | grep wlp | grep inet | awk '{print $2}' | awk  'BEGIN{OFS=FS="/"};NF--');
if [ -n "$IP" ]; then
    export ROS_IP=$IP;
fi


# master node is run on Jetson
export ROS_MASTER_URI=http://192.168.0.105:11311
