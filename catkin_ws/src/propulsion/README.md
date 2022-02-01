# Propulsion


## Overview


The propulsion package is responsible for moving the AUV and provides a hardware-agnostic interface.

### License

The source code is released under a [MIT license](../../../LICENSE).

The propulsion package has been tested under [ROS] Noetic opropulsionn Ubuntu 20.04.


## Package Interface

### Published Topics

*none*

### Subscribed Topics

| Topic | Message | description |
| ------ | ------- | ---------- |
| `\effort` | `geometry_msgs/Wrench` | Force and torque, relative to the robot's frame of reference to be applied at a given moment |


## Installation

### Dependencies

- `catkin`
- `geometry_msgs`
- `propulsion_msgs`
- `rosserial_arduino`
- `rosserial_client`

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

	rostopic pub -1 /effort geometry_msgs/Wrench.msg "force: {x: 1.0, y: 0.0, z: -0.5}, torque: {x: 1.0, y: -0.5, z: -2.0}"
	
