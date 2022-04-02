# Planner

## Overview

The planner package is responsible for making the high-level decision regarding what the AUV 
will do depending on its current 'operational state'

### License

The source code is released under a [MIT license](LICENSE.md).

The planner package has been tested under [ROS] Noetic on Ubuntu 20.04.


## Package Interface

### Actions

This package has a Waypoint action client (as specified in auv_msgs).

| Action part | Message | description |
| ------ | ------- | ---------- |
| target | `geometry_msgs/Pose` | Pose (global ref. frame) you want the AUV to assume |
| feedback | `geometry_msgs/Pose` | Current pose of AUV (global ref. frame)|
| result | `geometry_msgs/Pose` | The final resulting pose of the AUV following the action - should be within permissible range of target |


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

The planner package has several 'plans' depending on the context in which the AUV is run.

#### To execute the RoboSub2022 mission (under development):

	roslaunch planner mission.launch
  
This will block waiting on a Waypoint action server to publish the first target to.
The most convenient way to get around this is to also launch the controls package which
provides the Waypoint server.

As of now, the mission comprises of submerging the AUV to a depth of 4.0 m.
	
### Usage

Stubbing/publishing a feedback message (geometry_msgs/Pose) from the action server
specifying the current state of the AUV:

	rostopic pub -1 /waypoint_server/feedback auv_msgs/WaypointActionFeedback \
	"header:
	  seq: 0
	  stamp:
	    secs: 0
	    nsecs: 0
	  frame_id: ''
	status:
	  goal_id:
	    stamp:
	      secs: 0
	      nsecs: 0
	    id: ''
	  status: 1
	  text: 'still goin'
	feedback:
	  curr_state: 
	    position:
	      x: 0.0
	      y: 0.0
	      z: -1.0
	    orientation:
	      x: 0.0
	      y: 0.0
	      z: 0.0
	      w: 1.0
	" 
 
Stubbing/publishing a result message (geometry_msgs/Pose) from the action server
specifying the final state of the AUV (completion of action):

	rostopic pub -1 /waypoint_server/result auv_msgs/WaypointActionResult \
	"header:
	  seq: 0
	  stamp:
	    secs: 0
	    nsecs: 0
	  frame_id: ''
	status:
	  goal_id:
	    stamp:
	      secs: 0
	      nsecs: 0
	    id: ''
	  status: 1
	  text: 'success'
	result:
	  result_state: 
	    position:
	      x: 0.0
	      y: 0.0
	      z: -3.97
	    orientation:
	      x: 0.0
	      y: 0.0
	      z: 0.0
	      w: 1.0
	" 
  
 Look for debug messages indicating current state of operation. After mission completion, the 
 mission node should exit.
