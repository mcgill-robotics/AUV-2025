#!/usr/bin/env python
import rospy
from std_msgs.msg import String 

def sub_cb(msg):
    sentence = msg.data + "Nathaniel"
    pub.publish(String(sentence))
 
if __name__ == '__main__':
    rospy.init_node('node_two')
    pub = rospy.Publisher('complete', String, queue_size=1)
    sub = rospy.Subscriber('incomplete', String, sub_cb)
    rospy.spin()