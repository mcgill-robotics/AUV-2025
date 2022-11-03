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


 
pub = rospy.Publisher('/surge', Float64,queue_size=5)

set = Float64(0.0)
state = Float64(0.0)

def set_set(s):
        set = s
        pub.publish(Float64(set.data - state.data))

def set_state(s):
        state = s
        pub.publish(Float64(set.data - state.data))


if __name__ == '__main__':
        rospy.init_node('pid')
        rospy.Subscriber('/setpoint_x',Float64, set_set)
        rospy.Subscriber('/state_x',Float64, set_state)
        rospy.spin()
