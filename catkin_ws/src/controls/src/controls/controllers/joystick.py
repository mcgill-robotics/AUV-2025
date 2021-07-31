#!/usr/bin/env python

# Quick fix that subscribes to /cmd_vel
# and publishes to controls/wrench

import rospy
from geometry_msgs.msg import Twist, Wrench


class joystick:
    def __init__(self):
        self.wrench_msg = Wrench()

        self.sub = rospy.Subscriber('/cmd_vel',
                                    Twist,
                                    self.joy_cb,
                                    queue_size=1)

    def joy_cb(self, twist_msg):
        twist_msg.linear.y = -twist_msg.linear.y
        self.wrench_msg.force = twist_msg.linear
        twist_msg.angular.z = -twist_msg.angular.z
        self.wrench_msg.torque = twist_msg.angular


if __name__ == '__main__':
    rospy.init_node('teleop_node')

    js = joystick()
    pub = rospy.Publisher('/controls/wrench',
                          Wrench,
                          queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.logdebug(js.wrench_msg)
        pub.publish(js.wrench_msg)
        rate.sleep()
    rospy.spin()
