# Propulsion


## Overview


The propulsion package is responsible for moving the AUV and provides a hardware-agnostic interface.

### License

The source code is released under a [MIT license](propulsion/LICENSE).

The propulsion package has been tested under [ROS] Noetic opropulsionn Ubuntu 20.04.


## Package Interface

### Published Topics

*none*

### Subscribed Topics

| Topic | Message | description |
| ------ | ------- | ---------- |
| `\effort` | `geometry_msgs/Twist` | Force and torque, relative to the robot's frame of reference to be applied at a given moment |


## Installation

### Dependencies

- `catkin`
- `geometry_msgs`
- `propulsion_msgs`
- `rosserial_arduino`

### Building

	source /opt/ros/noetic/setup.bash
	cd <AUV-2020>/catkin_ws/src
	catkin build propulsion

After build is complete, make the packages visible to ROS

	source ../devel/setup.bash

### Running

Flash arduino

	catkin build --no-deps  propulsion --make-args propulsion_embedded_thrusters-upload

Launch all package nodes

	roslaunch propulson propulsion.launch
	
### Usage

Publishing a `geometry_msgs/Twist` message onto `/effort` topic:

	rospub /effort geometry_msgs/Twist.msg "[1, 1, 1], [0, 0, 0]"
	
---

## Implementation Details

TODO
