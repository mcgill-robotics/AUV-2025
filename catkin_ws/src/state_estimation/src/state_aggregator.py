#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import Float64
from state_variables import *


class State_Aggregator:
    pub_x = rospy.Publisher('state_x', Float64, queue_size=50)
    pub_y = rospy.Publisher('state_y', Float64, queue_size=50)
    pub_z = rospy.Publisher('state_z', Float64, queue_size=50)
    pub_theta_x = rospy.Publisher('state_theta_x', Float64, queue_size=50)
    pub_theta_y = rospy.Publisher('state_theta_y', Float64, queue_size=50)
    pub_theta_z = rospy.Publisher('state_theta_z', Float64, queue_size=50)


    def __init__(self):
        # position
        self.x = X()
        self.y = Y()
        self.z = Z()

        # orientation
        self.theta_x = Theta_X()
        self.theta_y = Theta_Y()
        self.theta_z = Theta_Z()


    def update_state(self, _):
        State_Aggregator.pub_x.publish(Float64(self.x.get())) 
        State_Aggregator.pub_y.publish(Float64(self.y.get())) 
        State_Aggregator.pub_z.publish(Float64(self.z.get())) 
        State_Aggregator.pub_theta_x.publish(Float64(self.theta_x.get())) 
        State_Aggregator.pub_theta_y.publish(Float64(self.theta_y.get())) 
        State_Aggregator.pub_theta_z.publish(Float64(self.theta_z.get())) 

if __name__ == '__main__':
    rospy.init_node('state_aggregator')
    sa = State_Aggregator()
    timer = rospy.Timer(rospy.Duration(0.1), sa.update_state)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
