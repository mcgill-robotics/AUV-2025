#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float64

pub_theta_y = rospy.Publisher('/theta_y_setpoint', Float64, queue_size=5, latch=True)
pub_z = rospy.Publisher('/z_setpoint', Float64, queue_size=5, latch=True)

if __name__ == '__main__':
	rospy.init_node('heave_calibration')
	
	# Move AUV down to submerge itself
	pub_z.publish(-0.5)
	
	# Want the AUV to level itself (such that theta_y = 0)
	pub_theta_y.publish(0.0)
	rospy.spin()
