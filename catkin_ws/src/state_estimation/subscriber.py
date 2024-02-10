#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16

def sub_cb(msg):
    print(msg.data + 1)
    
if __name__ == '__main__':
    rospy.init_node('subscriber')
    sub = rospy.Subscriber('odds', Int16, sub_cb)
    rospy.spin()