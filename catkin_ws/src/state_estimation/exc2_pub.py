#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def publisher():
    while not rospy.is_shutdown():
        pub.publish(String("My name is: "))

if __name__ == "__main__":
    rospy.init_node('publisher')
    pub = rospy.Publisher('exc2', String, queue_size=1)
    try:
        publisher()
    except rospy.ROSInterruptException:
        print("ROSInterruptException")
        pass
