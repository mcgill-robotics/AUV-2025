#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float64

pub = rospy.Publisher('/theta_y_setpoint', Float64, queue_size=5)

if __name__ == '__main__':
	rospy.init_node('heave_calibration')
	
	rospy.sleep(5)
	
	# Want to publish theta of y = 0 radians (level)
	pub.publish(0.0)
	rospy.spin()
