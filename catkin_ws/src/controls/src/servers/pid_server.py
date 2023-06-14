#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import Float64
from servers.base_server import BaseServer
import scipy
import actionlib
from auv_msgs.msg import StateAction


class PID(BaseServer):
    def __init__(self):
        self.establish_pid_publishers()
        self.establish_state_subscribers()
        self.server = actionlib.SimpleActionServer('state_server', StateAction, execute_cb= self.callback, auto_start = False)
        
        rospy.Subscriber('state_theta_x', Float64, self.updateThetaX)
        rospy.Subscriber('state_theta_y', Float64, self.updateThetaY)
        rospy.Subscriber('state_theta_z', Float64, self.updateThetaZ)
        rospy.Subscriber('theta_x_setpoint_adjusted', Float64, self.updateSetpointX)
        rospy.Subscriber('theta_y_setpoint_adjusted', Float64, self.updateSetpointY)
        rospy.Subscriber('theta_z_setpoint_adjusted', Float64, self.updateSetpointZ)
        
        self.theta_x = None
        self.theta_y = None
        self.theta_z = None
        self.q1 = None
        self.q2 = None
        self.setpoint_x = None
        self.setpoint_y = None
        self.setpoint_z = None
        self.Kp = [0, 0, 0, 1]
        self.Ki = [0, 0, 0, 1]
        self.Kd = [0, 0, 0, 1]
        self.error = [0, 0, 0, 1]
        self.integral_error = [0, 0, 0, 1]
        self.dt = 0
        self.derivative_error = [0, 0, 0, 1]
        
    def updateThetaX(self, msg):
        self.theta_x = float(msg.data)
    def updateThetaY(self, msg):
        self.theta_y = float(msg.data)
    def updateThetaZ(self, msg):
        self.theta_z = float(msg.data)
        
    def updateSetpointX(self, msg):
        self.setpoint_x = float(msg.data)
    def updateSetpointY(self, msg):
        self.setpoint_y = float(msg.data)
    def updateSetpointZ(self, msg):
        self.setpoint_z = float(msg.data)
        
    def convertPose(self):
        self.q1 = tf.transformations.quaternion_from_euler(self.theta_x, self.theta_y, self.theta_z)
    def convertDestination(self):
        self.q2 = tf.transformations.quaternion_from_euler(self.setpoint_x, self.setpoint_y, self.setpoint_z)         
    
    def updateError(self):
        q1_inv = [-self.q1[0], -self.q1[1], -self.q1[2], self.q1[3]]
        qr = tf.transformations.quaternion_multiply(q1_inv, self.q2)
        # some calculation
        self.error = qr
        
    def controlEffort(self):
        proportional = self.Kp * self.error
        integration = self.Ki * (self.integral_error + self.error * self.dt)
        derivative = self.Kd * self.derivative_error 
        control_effort = proportional + integration + derivative 
        pub_control_effort.publish(control_effort)
        

if __name__ == '__main__':
    rospy.init_node('PID', log_level=rospy.DEBUG)
    pid = PID()
    pub_control_effort = rospy.Publisher('[CONTROL_EFFORT]', Float64, queue_size=50)