#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import Imu
from sbg_driver.msg import SbgEkfQuat, SbgImuData
from servers.base_server import BaseServer
import actionlib
from auv_msgs.msg import QuaternionAction
import numpy as np
import quaternion
import time


class QuaternionServer(BaseServer):

    def __init__(self):
        print("making quaternion server")
        self.server = actionlib.SimpleActionServer('quaternion_server', QuaternionAction, execute_cb=self.callback, auto_start=False)
        
        # Publishers
        self.pub_roll = rospy.Publisher('roll', Float64, queue_size=50)
        self.pub_pitch = rospy.Publisher('pitch', Float64, queue_size=50)
        self.pub_yaw = rospy.Publisher('yaw', Float64, queue_size=50)
        self.pub_surge = rospy.Publisher('surge', Float64, queue_size=50)
        self.pub_sway = rospy.Publisher('sway', Float64, queue_size=50)
        self.pub_heave = rospy.Publisher('heave', Float64, queue_size=50)
        
        # Subscribers
        pose_sub = rospy.Subscriber('pose', Pose, self.pose_callback)
        imu_sub = rospy.Subscriber("/sbg/imu_data", SbgImuData, self.imu_callback)
        
        # Calculation parameters/values
        self.pose = Pose()
        self.body_quat = []
        self.position = []
        
        self.Kp = [0, 0, 0, 1]
        self.Ki = [0, 0, 0, 1]
        self.Kd = [0, 0, 0, 1]
        self.integral_error = [0, 0, 0, 1]
        self.dt = [0, 0] 
        self.angular_velocity = [0, 0, 0]
        self.server.start()        
        
    def pose_callback(self, data):
        # Assign pose values
        self.pose = data
        self.position = [data.position.x, data.position.y, data.position.z]
        self.body_quat = [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
        
        # Get the current time to calculate delta 
        self.dt.append(rospy.get_rostime().to_sec())
        self.dt.pop(0) # only need the two most recent two
    
    def imu_callback(self, data):
        self.ang_velocity = [data.gyro.x, data.gyro.y, data.gyro.z]
    
    def cancel(self):
        self.controlEffort(self.pose)
    
    def callback(self, goal):
        if self.pose != None:
            if(goal.displace):
                # displaced_position, displace_quat = self.get_goal_after_displace(goal.pose)
                displace_quat = self.get_goal_after_displace(goal.pose)
                displaced_position = []
                self.controlEffort(displaced_position, displace_quat)
            else:
                goal_position = []
                goal_position.append(goal.pose.position.x)
                goal_position.append(goal.pose.position.y)
                goal_position.append(goal.pose.position.z)
                goal_quat = [goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w]
                self.controlEffort(goal_position, goal_quat)
        
        # monitor when reached pose
        self.wait_for_settled()

        self.server.set_succeeded()
            
    def wait_for_settled(self):
        interval = 4
        settled = False
        print("waiting for settled")
        while not settled and not self.cancelled:
            start = time.time()
            while not self.cancelled and self.check_status():
                if(time.time() - start > interval):
                    settled = True
                    break
                rospy.sleep(0.01)
        print("settled") 
        
    def check_status(self):
        if(self.position == None or self.body_quat == None):
            return False

        # tolerance_position = 0.5
        tolerance_orientation = 0.009

        # x_diff = (not self.goal.do_x.data) or abs(self.position.x - self.goal.position.x) <= tolerance_position
        # y_diff = (not self.goal.do_y.data) or abs(self.position.y - self.goal.position.y) <= tolerance_position
        # z_diff = (not self.goal.do_z.data) or abs(self.position.z - self.goal.position.z) <= tolerance_position
    
        quat_diff = True
        error_quat = self.calculateError(self.body_quat, self.goal.pose.orientation)
        for i in range(3):
            if abs(error_quat[i+1] - self.body_quat[i+1]) > tolerance_orientation:
                quat_diff = False
        else:
            quat_diff = False
        
        # return x_diff and y_diff and z_diff and theta_x_diff and theta_y_diff and theta_z_diff
        return quat_diff
        
    def get_goal_after_displace(self, goal_pose):
        # new_goal = Pose()
        # goal_position = [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z]
        displacement_quat = [goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w]
        
        new_goal_quat = tf.transformations.quaternion_multiply(self.body_quat, displacement_quat)
        # return goal_position, new_goal_quat 
        return new_goal_quat
    
    def calculateError(self, q1, q2):
        q1_inv = [-q1[0], -q1[1], -q1[2], q1[3]]
        error = tf.transformations.quaternion_multiply(q1_inv, q2)
        return error
    
    def calculateDerivativeError(self, error, ang_velocity):
        velocity_quat = [ang_velocity[0], ang_velocity[1], ang_velocity[2], 0]
        return tf.transformations.quaternion_multiply(error, velocity_quat) / (-2)
    
    def calculateIntegralError(self, integral_error, error, delta_time):
        return integral_error + (error * delta_time)
        
    def controlEffort(self, goal_position, goal_quat):
        delta_time = (self.dt[1] - self.dt[0])
        
        # Calculate error values
        error = self.calculateError(self.body_quat, goal_quat) 
        derivative_error = self.calculateDerivativeError(error, self.angular_velocity)
        self.integral_error = self.calculateIntegralError(self.integral_error, error, delta_time)
        
        # Calculate proportional term
        proportional = np.multiply(self.Kp, error)
        # Calculte integral term
        integration = np.multiply(self.Ki, self.integral_error)
        # Calculate derivative term
        derivative = np.multiply(self.Kd, derivative_error)
        
        print(proportional)
        print(integration)
        print(derivative)
        
        control_effort = proportional + integration + derivative
        print(control_effort)
        control_effort_quat = np.quaternion(control_effort[0], control_effort[1], control_effort[2], control_effort[3])
        vector_3d = quaternion.as_rotation_vector(control_effort_quat)
        
        inertial_matrix = np.array([[0.042999259180866,  0.000000000000000, -0.016440893216213],
                                    [0.000000000000000,  0.709487776484284, 0.003794052280665], 
                                    [-0.016440893216213, 0.003794052280665, 0.727193353794052]])
        
        torque = np.matmul(inertial_matrix, vector_3d)
        
        self.pub_roll.publish(torque[0])
        self.pub_pitch.publish(torque[1])
        self.pub_yaw.publish(torque[2])
        self.pub_surge.publish(0.0)
        self.pub_sway.publish(0.0)
        self.pub_heave.publish(0.0)
        
    
        
