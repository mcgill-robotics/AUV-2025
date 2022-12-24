#!/usr/bin/env python3

import rospy
import math

from std_msgs.msg import Float64

pub_theta_x = rospy.Publisher('/theta_x_setpoint', Float64, queue_size=5, latch=True)
pub_theta_y = rospy.Publisher('/theta_y_setpoint', Float64, queue_size=5, latch=True)
pub_theta_z = rospy.Publisher('/theta_z_setpoint', Float64, queue_size=5, latch=True)

pub_z = rospy.Publisher('/z_setpoint', Float64, queue_size=5, latch=True)

if __name__ == '__main__':
	rospy.init_node('rolled_orientation_calibration')
	
	# Move AUV down to submerge itself
	pub_z.publish(-0.5)
	
	# Rolled up by 90 degrees
	pub_theta_x.publish(math.pi/2)
	
	# Want the AUV to level itself (such that theta_y = 0)
	pub_theta_y.publish(0.0)
	
	# Want 0 degree yaw
	pub_theta_z.publish(0.0)
	
	rospy.spin()