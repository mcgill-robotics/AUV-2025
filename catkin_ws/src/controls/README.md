# Controls

## Overview


The controls package is responsible for determining the effort the AUV should exert at a given moment in time based on a target and the current state.

### License

The source code is released under a GPLv3 license.

## Package Interface

### Published Topics

| Topic | Message | description |
| ------ | ------- | ---------- |
| `effort` | `geometry_msgs/Wrench` | Force and torque (N and N\*m), relative to the robot's frame of reference to be applied at a given moment |

### Subscribed Topics

| Topic | Message | description |
| ------ | ------- | ---------- |
| `state` | `geometry_msgs/Pose` | The current estimate of the position/orientation (pose) of the AUV |

### Actions

This package implements a Waypoint action server as specified in auv_msgs.

| Action part | Message | description |
| ------ | ------- | ---------- |
| goal | `geometry_msgs/Pose` | Pose (global ref. frame) you want the AUV to assume |
| feedback | `geometry_msgs/Pose` | Current pose of AUV (global ref. frame)|
| result | `geometry_msgs/Pose` | The final resulting pose of the AUV following the action - should be within permissible range of target |


## Installation

### Dependencies

- `auv_msgs`
- `catkin`
- `geometry_msgs`
- `pid`
- `rospy`
- `std_msgs`
- `tf`

### Building

	source /opt/ros/noetic/setup.bash
	cd <AUV-2020>/catkin_ws/src
	catkin build controls

After build is complete, make the packages visible to ROS

	source ../devel/setup.bash

### Running

Launch all package nodes

	roslaunch controls controls.launch
