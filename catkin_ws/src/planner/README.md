# Planner

## Overview

The planner package is responsible for making the high-level decision regarding what the AUV 
will do depending on its current 'operational state'

The planner package has been tested under ROS Noetic for Ubuntu 20.04.

### License

The source code is released under a GPLv3 license.

## Package Interface

### Actions

| Action part | Message | description |
| ------ | ------- | ---------- |
| todo | `todo` | todo |

## Installation

### Dependencies

- `auv_msgs`
- `catkin`
- `geometry_msgs`
- `rospy`
- `smach`
- `std_msgs`

### Building

	source /opt/ros/noetic/setup.bash
	cd <AUV-2020>/catkin_ws/src
	catkin build planner

After build is complete, make the packages visible to ROS

	source ../devel/setup.bash

### Running

The planner package has several 'missions' depending on the context in which the AUV is run.

#### To execute the RoboSub2022 mission (under development):

	roslaunch planner missions.launch
  
This will execute any missions which are called in src/mission.py

To add/remove missions to be executed, uncomment/comment out lines calling the mission state machines.
Several missions can be executed in order by calling them in the desired order.
A mission will only start when the previous one is done.

As of now, the missions comprise of testing the rotation of the AUV at 2m depth, and grid search then rotating according to any lane marker found.
	
### Usage

Add more missions by imitating the other missions already implemented (testRotation, navigateLaneMarker).

Ensure that the endMission() function is called after the mission completes.
