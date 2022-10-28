#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float64 

'''
This node implements a PID controller (only proportional term)
for the forward (x) direction of the robot

Assume that the robot is able to move along the x axis, 
and that it is always oriented forwards towards positive x 
(so you don't need to bother with coordinate transformations)

TO IMPLEMENT
---------------

Subscribers:
    1. to get setpoint from /setpoint_x topic (Float64)
    2. to get current state from /state_x topic (float64)

Publishers:
    1. output the force (effort) with which the robot should 
    move in the positive x direction on /surge topic
'''


if __name__ == '__main__':
    rospy.init_node('pid')
    rospy.spin()
