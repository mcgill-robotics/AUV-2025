#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion
from state_variables import *


class State_Aggregator:

    def __init__(self):
        # position
        self.x = X()
        self.y = Y()
        self.z = Z()

        # orientation
        self.theta_x = Theta_X()
        self.theta_y = Theta_Y()
        self.theta_z = Theta_Z()

        self.pub = rospy.Publisher('pose', Pose, queue_size=50)

        self.pub_x = rospy.Publisher('state_x', Float64, queue_size=50)
        self.pub_y = rospy.Publisher('state_y', Float64, queue_size=50)
        self.pub_z = rospy.Publisher('state_z', Float64, queue_size=50)
        self.pub_theta_x = rospy.Publisher('state_theta_x', Float64, queue_size=50)
        self.pub_theta_y = rospy.Publisher('state_theta_y', Float64, queue_size=50)
        self.pub_theta_z = rospy.Publisher('state_theta_z', Float64, queue_size=50)



    def update_state(self, _):
        # publish pose
        position = Point(
                self.x.get(),
                self.y.get(),
                self.z.get())
        quaternion = tf.transformations.quaternion_from_euler(
                self.theta_x.get(),
                self.theta_y.get(),
                self.theta_z.get())
        orientation = Quaternion(
                quaternion[0],
                quaternion[1],
                quaternion[2],
                quaternion[3])
       	pose  = Pose(position, orientation)
        self.pub.publish(pose)

        # publish individual degrees of freedom
        self.pub_x.publish(self.x.get())
        self.pub_y.publish(self.y.get())
        self.pub_z.publish(self.z.get())
        self.pub_theta_x.publish(self.theta_x.get())
        self.pub_theta_y.publish(self.theta_y.get())
        self.pub_theta_z.publish(self.theta_z.get())



if __name__ == '__main__':
    rospy.init_node('state_aggregator')
    sa = State_Aggregator()
    timer = rospy.Timer(rospy.Duration(0.1), sa.update_state)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
