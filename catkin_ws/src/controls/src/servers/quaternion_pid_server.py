#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from servers.base_server import BaseServer
import scipy
import actionlib
from auv_msgs.msg import QuaternionAction


class PID(BaseServer):

    def __init__(self):
        self.establish_pid_publishers()
        self.establish_state_subscribers()
        self.server = actionlib.SimpleActionServer('quaternion_server', QuaternionAction, execute_cb= self.callback, auto_start = False)
        
        self.goal = None
        self.pose = None
        pose_sub = rospy.Subscriber('pose', Pose, self.pose_callback)
        self.Kp = [0, 0, 0, 1]
        self.Ki = [0, 0, 0, 1]
        self.Kd = [0, 0, 0, 1]
        self.error = [0, 0, 0, 1]
        self.integral_error = [0, 0, 0, 1]
        self.dt = 0
        self.derivative_error = [0, 0, 0, 1]
        

    def pose_callback(self, data):
        self.pose = data.pose
        

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