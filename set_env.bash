#!/usr/bin/env bash

# set ROS_IP to the IP adress of this host machine
IP=$(ip r | grep ^192 | awk '{print $9}');
if [ -z '$IP' ]; then
    export ROS_IP=$IP;
fi


# master node is run on Jetson
export ROS_MASTER_URI=http://192.168.0.105:11311
