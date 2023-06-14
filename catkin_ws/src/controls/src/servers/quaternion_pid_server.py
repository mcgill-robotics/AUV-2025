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
import time

# as_rotation_vector


class QuaternionServer():

    def __init__(self):
        self.establish_pid_publishers()
        self.establish_state_subscribers()
        self.server = actionlib.SimpleActionServer('quaternion_server', QuaternionAction, execute_cb= self.callback, auto_start = False)
        
        self.goal = None
        self.body_quat = None
        self.position = None
        pose_sub = rospy.Subscriber('pose', Pose, self.pose_callback)
        self.Kp = [0, 0, 0, 1]
        self.Ki = [0, 0, 0, 1]
        self.Kd = [0, 0, 0, 1]
        self.integral_error = [0, 0, 0, 1]
        self.dt = 0
        self.control_effort_pub = rospy.Publisher('[something]', Float64, queue_size=50)
        self.angular_velocity = [0, 0, 0]
        # If sim:
        ang_velocity_sub = rospy.Subscriber('/imu', Something, self.ang_velocity_callback_sim)
        # If clarke:
        # ang_velocity_sub = rospy.Subscriber('something', Something, self.ang_velocity_callback_clarke)
        
        # Need someway to find DELTA Time (for derivative term)
        
    def pose_callback(self, data):
        self.position = [data.position.x, data.position.y, data.position.z]
        self.body_quat = np.quaternion(self.pose.orientation.w, self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z)
    
    def ang_velocity_callback_clarke(self, data):
        ###### Change depending on the topic ######
        self.ang_velocity = data
    
    def ang_velocity_callback_sim(self, data):
        ang_vel_x = data.angular_velocity.x
        ang_vel_y = data.angular_velocity.y
        ang_vel_z = data.angular_velocity.z
        self.ang_velocity = [ang_vel_x, ang_vel_y, ang_vel_z]
    
    def cancel(self):
        self.set_pids(self.pose)
    
    def callback(self, goal):
        if(goal.displace):
            displaced_position, displace_quat = self.get_goal_after_displace(goal.pose)
            self.set_pids(displaced_position, displace_quat)
        else:
            goal_position = []
            goal_position.append(goal.pose.position.x)
            goal_position.append(goal.pose.position.y)
            goal_position.append(goal.pose.position.z)
            goal_quat = np.quaternion(goal.pose.orientation.w, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z)
            self.set_pids(goal_position, goal_quat)
        
    def get_goal_after_displace(self, goal_pose):
        new_goal = Pose()
        goal_position = [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z]
        displacement_quat = np.quaternion(goal_pose.orientation.w, goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z)
        new_goal_quat = self.body_quat * displacement_quat
        return goal_position, new_goal_quat

    def convertDestination(self,eulers):
        theta_x , theta_y, theta_z = eulers
        self.q2 = tf.transformations.quaternion_from_euler(theta_x, theta_y, theta_z)         
    
    def calculateError(self, q1, q2):
        q1_inv = [q1[0], q1[1], q1[2], q1[3]]
        error = tf.transformations.quaternion_multiply(q1_inv, q2)
        return error
    
    def calculateDerivativeError(self, error, ang_velocity):
        velocity_quat = [0, ang_velocity[0], ang_velocity[1], ang_velocity[2]]
        # Might be using the wrong operation for multiplication
        return (error * velocity_quat) / (-2)
    
    def calculateIntegralError(self, error):
        return self.integral_error + (error * self.dt)
        
    def controlEffort(self):
        # Calculate error values
        error = self.calculateError(self.body_quat, self.goal)
        derivative_error = self.calculateDerivativeError(error, self.angular_velocity)
        self.integral_error = self.calculateIntegralError(error, self.dt)
        
        # Calculate proportional term
        proportional = self.Kp * error
        # Calculte integral term
        integration = self.Ki * (self.integral_error + error * self.dt)
        # Calculate derivative term
        derivative = self.Kd * derivative_error 
        
        control_effort = proportional + integration + derivative 
        vector = quaternion.as_rotation_vector(control_effort)
        self.control_effort_pub.publish(vector)
        
