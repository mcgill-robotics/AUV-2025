#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64



positive_pub = rospy.Publisher('heave', Float64, queue_size=50)
negative_pub = rospy.Publisher('heave', Float64, queue_size=50)
heave_up = False
heave_down = False


def heave_cb(joy_msg):
    if joy_msg.buttons[1] == 1 and joy_msg.buttons[5] == 1: #heave up
        heave_up = True
        heave_down = False
    elif joy_msg.buttons[0] == 1 and joy_msg.buttons[5] == 1: #heave down
        heave_up = False
        heave_down = True
    else: #neither button pressed
        heave_up = False
        heave_down = False
        positive_pub.publish(Float64(0.0))
        negative_pub.publish(Float64(0.0))

    if heave_up:
        positive_pub.publish(Float64(3.0))
    elif heave_down:
        negative_pub.publish(Float64(-3.0))

rospy.Subscriber('joy', Joy, heave_cb )

if __name__ == '__main__':
    rospy.init_node('joy_heave')
    rospy.Subscriber('joy', Joy, heave_cb )
    rospy.spin() 
    
