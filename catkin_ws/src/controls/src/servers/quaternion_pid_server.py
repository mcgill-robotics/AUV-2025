#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from servers.base_server import BaseServer
import actionlib
from auv_msgs.msg import QuaternionAction
import numpy as np
import quaternion


class QuaternionServer():

    def __init__(self):
        self.establish_pid_publishers()
        self.establish_state_subscribers()
        self.server = actionlib.SimpleActionServer('quaternion_server', QuaternionAction, execute_cb= self.callback, auto_start = False)
        
        self.goal = None
        self.body_quaternion = None
        self.position = None
        pose_sub = rospy.Subscriber('pose', Pose, self.pose_callback)
        self.Kp = [0, 0, 0, 1]
        self.Ki = [0, 0, 0, 1]
        self.Kd = [0, 0, 0, 1]
        self.error = [0, 0, 0, 1]
        self.integral_error = [0, 0, 0, 1]
        self.dt = 0
        self.derivative_error = [0, 0, 0, 1]
        

    def pose_callback(self, data):
        self.position = [data.position.x, data.position.y, data.position.z]
        self.body_quaternion = np.quaternion(self.pose.orientation.w, self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z)
        

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
    
    def cancel(self):
        self.set_pids(self.pose)
    
    def callback(self, goal):
        if(goal.displace):
            displaced = self.get_goal_after_displace(goal.pose)
            self.set_pids(displaced)
        else:
            self.set_pids(goal.pose)
        
    def get_goal_after_displace(self, goal_pose):
        new_goal = Pose()
        self.position = [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z]
        displacement_quat = np.quaternion(goal_pose.orientation.w, goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z)
        new_goal_quat = self.body_quaternion * displacement_quat
        

    def set_pids(self, goal_pose):
        pass
        

if __name__ == '__main__':
    rospy.init_node('PID', log_level=rospy.DEBUG)
    pid = PID()
    pub_control_effort = rospy.Publisher('[CONTROL_EFFORT]', Float64, queue_size=50)