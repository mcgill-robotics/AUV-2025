#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

if __name__ == '__main__':

    rospy.init_node('test_tethys')
    pub = rospy.Publisher('model/tethys/joint/propeller_joint/cmd_pos', Float64, queue_size=50)
    while True:
        pub.publish(-30.0)
        rospy.sleep(8)
        pub.publish(30.0)
        rospy.sleep(8)
