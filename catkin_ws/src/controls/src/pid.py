#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float64 

'''
This node implements a PID controller (only proportional term)
for the forward (x) direction of the robot

We assume that the robot is able to move along the x axis, 
and that it is always oriented forwards towards positive x 
(so you don't need to bother with coordinate transformations)

'''

pub = rospy.Publisher('/surge', Float64, queue_size=5)

global current_setpoint
current_setpoint = None

def setCurrentSetpoint(setpt):
	global current_setpoint
	current_setpoint = setpt

def calculateEffortToMoveToSetPoint(current_x):
	global current_setpoint

	# Want to check that current_setpoint has a value
	if current_setpoint == None: return

	# Find distance between the desired location and current location
	# this is the error component Err
	err = current_setpoint.data - current_x.data
	
	# kP, the proportional gain that we multiply by the error
	proportional_gain = 0.2
	
	# Determine the effort (on x-axis, no rotation) needed for this 
	# multiplying the distance between x and setpoint (err) by the 
	# proportional gain value
	effort = err * proportional_gain
	
	pub.publish(effort)

if __name__ == '__main__':
    rospy.init_node('pid')
    sub = rospy.Subscriber('/setpoint_x', Float64, setCurrentSetpoint)
    sub = rospy.Subscriber('/state_x', Float64, calculateEffortToMoveToSetPoint)
    rospy.spin()
