#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

class StateTracker:
    def __init__(self):
        self.x_pos_sub = rospy.Subscriber('state_x', Float64, self.updateX)
        self.y_pos_sub = rospy.Subscriber('state_y', Float64, self.updateY)
        self.z_pos_sub = rospy.Subscriber('state_z', Float64, self.updateZ)
        self.theta_x_sub = rospy.Subscriber('state_theta_x', Float64, self.updateThetaX)
        self.theta_y_sub = rospy.Subscriber('state_theta_y', Float64, self.updateThetaY)
        self.theta_z_sub = rospy.Subscriber('state_theta_z', Float64, self.updateThetaZ)
        self.state_x = None
        self.state_y = None
        self.state_z = None
        self.state_theta_x = None
        self.state_theta_y = None
        self.state_theta_z = None
    def updateX(self, msg):
        self.state_x = float(msg.data)
    def updateY(self, msg):
        self.state_y = float(msg.data)
    def updateZ(self, msg):
        self.state_z = float(msg.data)
    def updateThetaX(self, msg):
        self.state_theta_x = float(msg.data)
    def updateThetaY(self, msg):
        self.state_theta_y = float(msg.data)
    def updateThetaZ(self, msg):
        self.state_theta_z = float(msg.data)
    def getPosition(self):
        return (self.state_x, self.state_y, self.state_z)
    def getRotation(self):
        return (self.state_theta_x, self.state_theta_y, self.state_theta_z)
    def stop(self):
        self.x_pos_sub.unregister()
        self.y_pos_sub.unregister()
        self.z_pos_sub.unregister()
        self.theta_x_sub.unregister()
        self.theta_y_sub.unregister()
        self.theta_z_sub.unregister()
