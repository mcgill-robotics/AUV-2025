#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import Imu
from sbg_driver.msg import SbgEkfQuat, SbgImuData
from servers.base_server import BaseServer
from math import pow
import actionlib
from auv_msgs.msg import QuaternionAction
import numpy as np
import quaternion
import time


class QuaternionServer(BaseServer):

    def __init__(self):
        super().__init__()
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
        
        self.Kp = 1
        self.Ki = 1
        self.Kd = 1
        self.integral_error_quat = [0, 0, 0, 1]
        self.integral_error_pos = [1, 1, 1]
        self.dt = [0, 0] 
        self.angular_velocity = [0, 0, 0]
        self.linear_velocity = [0, 0, 0]
        self.server.start()        
        
    def pose_callback(self, data):
        # Assign pose values
        self.pose = data
        self.position = [data.position.x, data.position.y, data.position.z]
        self.body_quat = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        
        # Get the current time to calculate delta 
        self.dt.append(rospy.get_rostime().to_sec())
        self.dt.pop(0) # only need the two most recent two
    
    def imu_callback(self, data):
        self.ang_velocity = [data.gyro.x, data.gyro.y, data.gyro.z]
    
    def cancel(self):
        self.controlEffort(self.position, self.body_quat)
    
    def callback(self, goal):
        if self.pose != None:
            if(goal.displace):
                displaced_position, displace_quat = self.get_goal_after_displace(goal.pose)
                self.controlEffort(displaced_position, displace_quat)
            else:
                goal_position = [goal.position.x, goal.position.y, goal.position.z]
                goal_quat = [goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w]
                self.controlEffort(goal_position, goal_quat)
        
            # monitor when reached pose
            self.wait_for_settled(goal)

            self.server.set_succeeded()
                
    def wait_for_settled(self, goal):
        interval = 4
        settled = False
        print("waiting for settled")
        while not settled and not self.cancelled:
            start = time.time()
            while not self.cancelled and self.check_status(goal):
                if(time.time() - start > interval):
                    settled = True
                    break
                rospy.sleep(0.01)
        print("settled") 
        
    def check_status(self, goal):
        if(self.position == None or self.body_quat == None):
            return False

        tolerance_position = 0.5
        tolerance_orientation = 0.05

        x_diff = (not self.goal.do_x.data) or abs(self.position.x - self.goal.position.x) <= tolerance_position
        y_diff = (not self.goal.do_y.data) or abs(self.position.y - self.goal.position.y) <= tolerance_position
        z_diff = (not self.goal.do_z.data) or abs(self.position.z - self.goal.position.z) <= tolerance_position
        x, y, z, w = goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w
        innert_product = self.body_quat[0]*x + self.body_quat[1]*y + self.body_quat[2]*z + self.body_quat[3]*w
            
        theta = np.arccos(2 * pow(innert_product, 2) - 1)
        quat_diff = True
        
        if abs(theta) > tolerance_orientation:
            quat_diff = False
        
        return x_diff and y_diff and z_diff and quat_diff
        
    def get_goal_after_displace(self, goal_pose):
        # new_goal = Pose()
        goal_position = [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z]
        displacement_quat = [goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w]
        
        new_goal_quat = tf.transformations.quaternion_multiply(self.body_quat, displacement_quat)
        return goal_position, new_goal_quat 
    
    def calculateError(self, q1, q2, p1, p2):
        q1_inv = [-q1[0], -q1[1], -q1[2], q1[3]]
        error_quat = tf.transformations.quaternion_multiply(q1_inv, q2)
        error_pos = []
        for i in range(3):
            error_pos.append(p2[i] - p1[i])
        return error_quat, error_pos
    
    def calculateDerivativeError(self, error, ang_velocity):
        velocity_quat = [ang_velocity[0], ang_velocity[1], ang_velocity[2], 0]
        return tf.transformations.quaternion_multiply(error, velocity_quat) / (-2)
    
    def calculateIntegralError(self, integral_error_quat, integral_error_pos, error_quat, error_pos, delta_time):
        return integral_error_quat + (error_quat * delta_time), integral_error_pos + (error_pos * delta_time)
        
    def controlEffort(self, goal_position, goal_quat):
        delta_time = (self.dt[1] - self.dt[0])
        
        # Calculate error values
        error_quat, error_pos = self.calculateError(self.body_quat, goal_quat, self.position, goal_position) 
        derivative_error_quat, derivative_error_pos = self.calculateDerivativeError(error_quat, self.angular_velocity, error_pos, self.linear_velocity)
        self.integral_error_quat, self.integral_error_pos = self.calculateIntegralError(self.integral_error_quat, self.integral_error_pos, error_quat, error_pos, delta_time)
        
        proportional_quat, integration_quat, derivative_quat = [], [], []
        proportional_pos, integration_pos, derivative_pos = [], [], []
        for i in range(3):
            # Calculate proportional term
            proportional_quat.append(self.Kp * error_quat[i])
            proportional_pos.append(self.Kp * error_pos[i])
            # Calculte integral term
            integration_quat.append(self.Ki * self.integral_error_quat[i])
            integration_pos.append(self.Ki * self.integral_error_pos[i])
            # Calculate derivative term
            derivative_quat.append(self.Kd * derivative_error_quat[i])
            derivative_pos.append(self.Kd * derivative_error_pos[i])
            
        control_effort_pos = proportional_pos + integration_pos + derivative_pos
        
        control_effort_quat = proportional_quat + integration_quat + derivative_quat
        control_effort_quat = np.quaternion(control_effort_quat[0], control_effort_quat[1], control_effort_quat[2], control_effort_quat[3])
        vector_3d = quaternion.as_rotation_vector(control_effort_quat)
        
        inertial_matrix = np.array([[0.042999259180866,  0.000000000000000, -0.016440893216213],
                                    [0.000000000000000,  0.709487776484284, 0.003794052280665], 
                                    [-0.016440893216213, 0.003794052280665, 0.727193353794052]])
        
        torque = np.matmul(inertial_matrix, vector_3d)
        
        self.pub_roll.publish(torque[0])
        self.pub_pitch.publish(torque[1])
        self.pub_yaw.publish(torque[2])
        self.pub_surge.publish(control_effort_pos[0])
        self.pub_sway.publish(control_effort_pos[1])
        self.pub_heave.publish(control_effort_pos[2])
        
    
        
