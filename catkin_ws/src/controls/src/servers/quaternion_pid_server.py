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


class QuaternionServer(BaseServer):

    def __init__(self):
        super().__init__()
        print("making quaternion server")
        self.establish_pid_publishers()
        self.establish_pid_enable_publishers()
        self.server = actionlib.SimpleActionServer('quaternion_server', QuaternionAction, execute_cb=self.callback, auto_start=False)
        
        # Publishers
        self.pub_roll = rospy.Publisher('roll', Float64, queue_size=50)
        self.pub_pitch = rospy.Publisher('pitch', Float64, queue_size=50)
        self.pub_yaw = rospy.Publisher('yaw', Float64, queue_size=50)
        
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
        self.time_interval = [0, rospy.get_time()] 
        self.angular_velocity = [0, 0, 0]
        self.server.start()        
        
    def pose_callback(self, data):
        # Assign pose values
        self.pose = data
        self.position = [data.position.x, data.position.y, data.position.z]
        self.body_quat = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        self.update_time_interval()

    def update_time_interval(self):
        self.time_interval[0] = self.time_interval[1]
        self.time_interval[1] = rospy.get_time()
    
    def imu_callback(self, data):
        self.ang_velocity = [data.gyro.x, data.gyro.y, data.gyro.z]
    
    def cancel(self):
        self.controlEffort(self.body_quat)
    
    def callback(self, goal):
        self.goal = goal
        if self.pose != None:
            if(goal.displace):
                displaced_position, displace_quat = self.get_goal_after_displace(goal.pose)
                self.execute_goal(displaced_position, displace_quat)
            else:
                goal_position = [goal.position.x, goal.position.y, goal.position.z]
                goal_quat = [goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w]
                self.execute_goal(goal_position, goal_quat)
        
            # monitor when reached pose
            self.server.set_succeeded()

    def execute_goal(self, goal_position, goal_quaternion):
        self.publish_position_pids(goal_position)
        interval = 4
        settled = False
        print("waiting for settled")
        rate = rospy.Rate(100)  
        while not settled and not self.cancelled:
            start = rospy.get_time()
            self.controlEffort(goal_quaternion)
            while not self.cancelled and self.check_status():
                self.controlEffort(goal_quaternion)
                if(rospy.get_time() - start > interval):
                    settled = True
                    break
                rate.sleep()
        print("settled")      
    
    def publish_position_pids(self, goal_position):
        if(self.goal.do_x.data):
            self.pid_x_enable.publish(True)
            self.pub_x_pid.publish(goal_position[0])
        if(self.goal.do_y.data):
            self.pid_y_enable.publish(True)
            self.pub_y_pid.publish(goal_position[1])
        if(self.goal.do_z.data):
            self.pid_z_enable.publish(True)
            self.pub_z_pid.publish(goal_position[2])
        
    def check_status(self, goal_position, goal_quaternion):
        if(self.position == None or self.body_quat == None):
            return False

        tolerance_position = 0.5
        tolerance_orientation = 0.05

        x_diff = (not self.goal.do_x.data) or abs(self.position.x - goal_position[0]) <= tolerance_position
        y_diff = (not self.goal.do_y.data) or abs(self.position.y - goal_position[1]) <= tolerance_position
        z_diff = (not self.goal.do_z.data) or abs(self.position.z - goal_position[2]) <= tolerance_position
        
        if self.goal.do_orientation.data:
            x, y, z, w = goal_quaternion
            innert_product = self.body_quat[0]*x + self.body_quat[1]*y + self.body_quat[2]*z + self.body_quat[3]*w
                
            theta = np.arccos(2 * pow(innert_product, 2) - 1)
            quat_diff = True
            
            if abs(theta) > tolerance_orientation:
                quat_diff = False
            else:
                quat_diff = True
        
        return x_diff and y_diff and z_diff and quat_diff
        
    def get_goal_after_displace(self, goal_pose):
        # new_goal = Pose()
        goal_position = [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z]
        displacement_quat = [goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w]
        
        new_goal_quat = tf.transformations.quaternion_multiply(self.body_quat, displacement_quat)
        return goal_position, new_goal_quat 
    
    def calculateError(self, q1, q2):
        q1_inv = [-q1[0], -q1[1], -q1[2], q1[3]]
        error_quat = tf.transformations.quaternion_multiply(q1_inv, q2)
        return error_quat
    
    def calculateDerivativeError(self, error, ang_velocity):
        velocity_quat = [ang_velocity[0], ang_velocity[1], ang_velocity[2], 0]
        return tf.transformations.quaternion_multiply(error, velocity_quat) / (-2)
    
    def calculateIntegralError(self, integral_error_quat, error_quat, delta_time):
        return integral_error_quat + (error_quat * delta_time)
    
    def kinematic_inversion(self,qe_des,qe):
        Qe2 = self.Qe(qe)
        matrix = np.transpose(Qe2) * -1
        return np.matmul(matrix, qe_des) * 2
    
    def Qe(self,qe):
        return np.array([[-qe[1], -qe[2], -qe[3]],
                         [qe[0],   qe[3], -qe[2]],
                         [-qe[3],  qe[0],  qe[1]],
                         [qe[2],  -qe[1],  qe[0]]])
    
    def orthogonal_matrix(self, q):
        Qe1 = self.Qe(q)
        transpose = np.transpose(Qe1.copy())
        return np.matmul(Qe1, transpose)
        
    def controlEffort(self, goal_quat):
        delta_time = (self.time_interval[1] - self.time_interval[0])
        
        # Calculate error values
        error_quat = self.calculateError(self.body_quat, goal_quat) 
        derivative_error_quat = self.calculateDerivativeError(error_quat, self.angular_velocity)
        self.integral_error_quat = self.calculateIntegralError(self.integral_error_quat, error_quat, delta_time)
        
        proportional_quat, integration_quat, derivative_quat = [], [], []
        for i in range(4):
            # Calculate proportional term
            proportional_quat.append(self.Kp * error_quat[i])
            # Calculte integral term
            integration_quat.append(self.Ki * self.integral_error_quat[i])
            # Calculate derivative term
            derivative_quat.append(self.Kd * derivative_error_quat[i])
            

        control_effort_quat = proportional_quat + integration_quat + derivative_quat
        # OPTIONAL control_effort_quat = np.matmul(self.orthogonal_matrix(error_quat), control_effort_quat
        # ISSUE: error_quat needs to be w,x,y,z for forumla to work
        omega_command = self.kinematic_inversion(control_effort_quat, error_quat)
        
        inertial_matrix = np.array([[0.042999259180866,  0.000000000000000, -0.016440893216213],
                                    [0.000000000000000,  0.709487776484284, 0.003794052280665], 
                                    [-0.016440893216213, 0.003794052280665, 0.727193353794052]])
        
        torque = np.matmul(inertial_matrix, omega_command)
        
        self.pub_roll.publish(torque[0])
        self.pub_pitch.publish(torque[1])
        self.pub_yaw.publish(torque[2])      
    
        
