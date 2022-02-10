#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64


if __name__ == '__main__':
    print('helloworld')
    rospy.init_node('depth')
    pub = rospy.Publisher('state', Float64, queue_size=50)
    data = Float64(3.14)
    while True:
        pub.publish(data)
    rospy.spin()
