# Propulsion

## Overview

The propulsion package is responsible for moving the AUV and provides a hardware-agnostic interface.

The package recieves wrenches from the controls package via `/controls/effort` and maps the force/torque wrenches to a thruster matrix on `/propulsion/forces`.

The package subsequently converts the forces per thruster to pwm on `propulsion/microseconds`.

The propulsion package has been tested under ROS Noetic for Ubuntu 20.04.

### License

The source code is released under a GPLv3 license.

## Package Interface

### Published Topics

| Topic | Message | description |
| ------ | ------- | ---------- |
| `/propulsion/microseconds` | `auv_msgs/ThrusterMicroseconds` | PWM values sent to the power board to spin thrusters with a desired duty cycle. |

### Subscribed Topics

| Topic | Message | description |
| ------ | ------- | ---------- |
| `/controls/effort` | `geometry_msgs/Wrench` | Force and torque, relative to the robot's frame of reference to be applied at a given moment. |

### Internal Topics

| Topic | Message | description |
| ------ | ------- | ---------- |
| `/propulsion/forces` | `auv_msgs/ThrusterForces` | Force each thruster must attain to move robot with the desired force and torque. |

## Installation

### Dependencies

- `catkin`
- `geometry_msgs`
- `auv_msgs`
- `python3-numpy`
- `python3-roscpp`
- `python3-rospy`
- `python3-keyboard`

### Building

	source /opt/ros/noetic/setup.bash
	cd AUV-2025/catkin_ws
	catkin build propulsion

After build is complete, make the packages visible to ROS

	source devel/setup.bash

### Running

Launch all package nodes

	roslaunch propulsion propulsion.launch

Launch dry test

	roslaunch propulsion drytest.launch

Launch wet test

	roslaunch propulsion wettest.launch
	
### Usage

The power board MCU must be mounted on `/dev/power`.

Publishing a `geometry_msgs/Wrench` message onto `/controls/effort` topic
	
	rostopic pub -1 /controls/effort geometry_msgs/Wrench "{force: {x: 1.0, y: 0.0, z: -0.5}, torque: {x: 1.0, y: -0.5, z: -2.0}}"

Echoing `/propulsion/microseconds`

	rostopic echo /propulsion/microseconds

Echoing `/propulsion/forces`

	rostopic echo /propulsion/forces
