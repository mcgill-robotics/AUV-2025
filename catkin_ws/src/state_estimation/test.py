#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16

def publisher():
    while not rospy.is_shutdown():
        pub.publish(Int16(1))
        rospy.sleep(1) # Delay to control publishing rate
        pub.publish(Int16(0))
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('publisher')
    pub = rospy.Publisher('odds', Int16)
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
