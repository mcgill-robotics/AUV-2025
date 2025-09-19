# Propulsion

The **propulsion** package handles low-level actuation for the AUV. It transforms high-level control efforts, expressed as [`geometry_msgs/Wrench`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Wrench.html), into **PWM signals (µs)** for each thruster.

This process occurs in two stages:

1. **Thrust allocation** – map the wrench vector (forces and torques along X, Y, Z) into individual thruster forces using an allocation matrix.  
2. **Force-to-PWM mapping** – convert thruster forces (in Newtons) into PWM signals (µs) using per-thruster calibration curves.  


## Table of Contents
- [Overview](#overview)
- [Usage](#usage)
- [Nodes](#nodes)
  - [Published Topics](#published-topics)
  - [Subscribed Topics](#subscribed-topics)
- [Installation](#installation)
  - [Dependencies](#dependencies)
  - [Building](#building)
  - [Running](#running)
- [License](#license)


## Overview
The [wrench](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Wrench.html) consists of forces AND torques on the X,Y,Z axes. These are distributed to the thrusters using the following allocation matrix:

<p align="center">
  <img src="https://github.com/user-attachments/assets/0a9ddce5-7d18-44d3-95cb-cef3be8d99f7" alt="Thruster Allocation Matrix" width="1000"/>
</p>


- Parameters a, b, c, d, e, and alpha describe the distances and angular offsets between thrusters and the AUV’s center of gravity.

- The *i-th* column corresponds to the contribution of thruster *i* to each element of the wrench vector.

- The *j-th* row corresponds to how all thrusters contribute to the *j-th* component of the wrench (force or torque along X, Y, Z).

<p align="center">
  <img src="https://github.com/user-attachments/assets/0a652533-93e6-4891-999d-b0c9fdebbf2f" width="800"/>
</p>

---

The resulting thruster forces (in Newtons) are converted to PWM microseconds using per-thruster calibration functions. These functions were obtained from a thruster test conducted by the Mech & Elec team in May 2025.

Comparison of calibration curves across thrusters:

<p align="center">
  <img src="https://github.com/user-attachments/assets/ce4abc5e-509c-4bb5-a4a2-2c947bb768be" alt="Thruster Allocation Matrix" width="800"/>
</p>


## Usage
The propulsion package is not for direct use, it is used through publishing efforts on the 'effort' topic.

Publishing a `geometry_msgs/Wrench` message onto `/controls/effort` topic:


	rostopic pub -1 /controls/effort geometry_msgs/Wrench "{force: {x: 1.0, y: 0.0, z: -0.5}, torque: {x: 1.0, y: -0.5, z: -2.0}}"


## Nodes
The package provides a single ROS node: `thrust_mapper`.

- Input: subscribes to `/controls/effort`

- Outputs: publishes thruster forces and PWM microseconds


### Published Topics

 Topic | Message | Description |
| ------ | ------- | ---------- |
| `/propulsion/forces` | `ThrusterForces` | Array of thruster forces (N) for each thruster  |
| `/propulsion/microseconds` | `ThrusterMicroseconds` | Array of PWM signals (µs) sent to each thrusterr |



### Subscribed Topics

| Topic | Message | Description |
| ------ | ------- | ---------- |
| `/controls/effort` | `geometry_msgs/Wrench` | Forces and torques, relative to the robot's frame of reference to be applied at a given moment |


## Installation

### Dependencies

- `catkin`
- `geometry_msgs`
- `propulsion_msgs`

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

	roslaunch propulsion propulsion.launch


### License

The source code is released under a GPLv3 license.
