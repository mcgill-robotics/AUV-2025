# Controls

## Overview


The controls package is responsible for determining the effort the AUV should exert at a given moment in time based on a target and the current state
Currently, only the depth is controlled.
### License

The source code is released under a [MIT license](propulsion/LICENSE).

The controls package has been tested under [ROS] Noetic opropulsionn Ubuntu 20.04.


## Package Interface

### Published Topics

| Topic | Message | description |
| ------ | ------- | ---------- |
| `effort` | `geometry_msgs/Wrench` | Force and torque (N and N\*m), relative to the robot's frame of reference to be applied at a given moment |

### Subscribed Topics

| Topic | Message | description |
| ------ | ------- | ---------- |
| `state` | `Float64` | The current depth (m) of the AUV |

### Actions

This package implements a PursueTarget action server as specified in auv_msgs.

| Action part | Message | description |
| ------ | ------- | ---------- |
| target | `Float64` | The desired depth (m) |
| feedback | `Float64` | The current depth (m) of the AUV |
| result | `Float64` | The final depth (m) of the AUV - should be within permissible range of target |


## Installation

### Dependencies

- `catkin`
- `geometry_msgs`
- `auv_msgs`
- `std_msgs`
- `rospy`
- `pid`

### Building

	source /opt/ros/noetic/setup.bash
	cd <AUV-2020>/catkin_ws/src
	catkin build controls

After build is complete, make the packages visible to ROS

	source ../devel/setup.bash

### Running

Launch all package nodes

	roslaunch controls controls.launch
	
### Usage

Publishing a `std_msgs/Float64` message onto `/pursueTarget/goal` topic:

	rostopic pub -1 /pursueTarget/goal std_msgs/Float64 4.0
  
 Publishing a `std_msgs/Float64` message onto `/state` topic:

	rostopic pub -1 /state std_msgs/Float64 0.0
  
 Look for messages published on /effort
