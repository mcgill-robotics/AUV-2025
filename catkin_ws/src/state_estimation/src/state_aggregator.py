#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Vector3
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

        self.pub = rospy.Publisher('state', Pose, queue_size=50)

    def update_state(self, _):
        # TODO - make better use of second param
        position = Vector3(
                self.x.get(), 
                self.y.get(), 
                self.z.get())
        orientation = Vector3(
                self.theta_x.get(), 
                self.theta_y.get(), 
                self.theta_z.get())
        pose  = Pose(position, orientation)
        self.pub.publish(pose)


if __name__ == '__main__':
    rospy.init_node('state_aggregator')
    sa = State_Aggregator()
    timer = rospy.Timer(rospy.Duration(0.1), sa.update_state)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
