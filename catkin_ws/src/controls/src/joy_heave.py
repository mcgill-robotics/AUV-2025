#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


pub = rospy.Publisher('/controls/force/heave', Float64, queue_size=1)

def heave_cb(joy_msg):
    if joy_msg.buttons[1] == 1 and joy_msg.buttons[5] == 1: #heave up
        pub.publish(Float64(3.0))
    elif joy_msg.buttons[0] == 1 and joy_msg.buttons[5] == 1: #heave down
        pub.publish(Float64(-3.0))
    else: #neither button pressed; reset heave
        pub.publish(Float64(0.0))

if __name__ == '__main__':
    rospy.init_node('joy_heave')
    rospy.Subscriber('joy', Joy, heave_cb )
    rospy.spin() 
    
