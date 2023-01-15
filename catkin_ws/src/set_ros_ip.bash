#!/usr/bin/env bash

# set ROS_IP to the IP adress of this host machine
export ROS_IP=$(ip r | grep ^192 | awk '{print $9}')

if [ -z "${ROS_MASTER_URI}" ];

    then export ROS_MASTER_URI=192.168.0.105:$1
fi
