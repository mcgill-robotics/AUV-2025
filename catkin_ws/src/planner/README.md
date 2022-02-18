# Planner

## Overview

The planner package is responsible for making the high-level decision regarding what the AUV 
will depending on its current 'operational state'

### License

The source code is released under a [MIT license](LICENSE.md).

The planner package has been tested under [ROS] Noetic on Ubuntu 20.04.


## Package Interface

### Actions

This package has a PursueTarget action client (as specified in auv_msgs).

| Action part | Message | description |
| ------ | ------- | ---------- |
| target | `Float64` | The desired depth (m) |
| feedback | `Float64` | The current depth (m) of the AUV |
| result | `Float64` | The final depth (m) of the AUV - should be within permissible range of target |


## Installation

### Dependencies

- `auv_msgs`
- `catkin`
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
  
This will block waiting on a PursueTarget action server to publish the first target to.
The most convenient way to get around this is to also launch the controls package which
provides the PursueTarget server.

As of now, the mission comprises of submerging the AUV to a depth of 4.0 m.
	
### Usage

Stubbing/publishing a feedback message (std_msgs/Float64) from the action server
specifying the current depth of the AUV:

	rostopic pub -1 /pursueTarget/feedback auv_msgs/PursueTargetActionFeedback "header:
	  seq: 0
	  stamp:
	    secs: 0
	    nsecs: 0
	  frame_id: ''
	goal_id:
	  stamp:
	    secs: 0
	    nsecs: 0
	  id: ''
	feedback:
	  curr_depth: 
	    data: 1.0
	" 
 
 Stubbing/publishing a result message (std_msgs/Float64) from the action server
specifying the final depth of the AUV (completion of action):

	rostopic pub -1 /pursueTarget/result auv_msgs/PursueTargetActionResult "header:
	  seq: 0
	  stamp:
	    secs: 0
	    nsecs: 0
	  frame_id: ''
	goal_id:
	  stamp:
	    secs: 0
	    nsecs: 0
	  id: ''
	result:
	  final_depth: 
	    data: 3.95
	" 
  
 Look for debug messages indicating current state of operation. After mission completion, the 
 mission node should exit.
 
 ### Issues
 
 - There are some issues having to do with timing the bringup of resources when launching planner followed by controls:
 The pid server is constantly waiting on the first setpoint even though this should be published following the first 
 action client request (immediately upon server start). After a bit of experimenting, adding a small time delay in the 
 controls action server constructor after creating setpoint publisher, before starting the server resolves this issue. 
 A less hacky solution is desireable. 
