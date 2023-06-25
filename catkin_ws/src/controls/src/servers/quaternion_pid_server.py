#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import Float64, Bool
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
        self.pub_roll = rospy.Publisher('roll', Float64, queue_size=1)
        self.pub_pitch = rospy.Publisher('pitch', Float64, queue_size=1)
        self.pub_yaw = rospy.Publisher('yaw', Float64, queue_size=1)
        
        # Subscribers
        pose_sub = rospy.Subscriber('pose', Pose, self.pose_callback)
        imu_sub = rospy.Subscriber("/sbg/imu_data", SbgImuData, self.imu_callback)
        # quat_sub = rospy.Subscriber("/sbg/ekf_quat", SbgEkfQuat, self.quat_callback)
        
        # Calculation parameters/values
        self.pose = None#Pose()
        self.body_quat = np.quaternion(1,0,0,0)
        self.position = []
        
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0.5
        self.integral_error_quat = np.quaternion()
        self.time_interval = [0, rospy.get_time()] 
        self.angular_velocity = np.zeros(3)
        self.server.start()        
        
    def pose_callback(self, data):
        # Assign pose values
        self.pose = data
        self.position = [data.position.x, data.position.y, data.position.z]
        self.body_quat = np.quaternion(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
        # self.update_time_interval()

    def update_time_interval(self):
        self.time_interval[0] = self.time_interval[1]
        self.time_interval[1] = rospy.get_time()
    
    def imu_callback(self, data):
        self.angular_velocity = np.array([-data.gyro.z, data.gyro.y, -data.gyro.x])


    def cancel(self):
        self.controlEffort(self.body_quat)
    
    def callback(self, goal):
        print("\n\nQuaternion Server got goal:\n",goal)
        self.goal = goal
        if self.pose != None:
            if(goal.displace):
                displaced_position, displace_quat = self.get_goal_after_displace(goal.pose)
                self.execute_goal(displaced_position, displace_quat)
            else:
                goal_position = [goal.position.x, goal.position.y, goal.position.z]
                goal_quat = np.quaternion(goal.orientation.w, goal.orientation.x, goal.orientation.y, goal.orientation.z)
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
            self.control_effort_dcm(goal_quaternion)
            while not self.cancelled and self.check_status(goal_position, goal_quaternion):
                self.control_effort_dcm(goal_quaternion)
                if(rospy.get_time() - start > interval):
                    settled = True
                    break
                rate.sleep()
        print("settled")      
    
    def publish_position_pids(self, goal_position):
        if(self.goal.do_x.data):
            self.pub_x_enable.publish(True)
            self.pub_x_pid.publish(goal_position[0])
        if(self.goal.do_y.data):
            self.pub_y_enable.publish(True)
            self.pub_y_pid.publish(goal_position[1])
        if(self.goal.do_z.data):
            self.pub_z_enable.publish(True)
            self.pub_z_pid.publish(goal_position[2])
        
    def check_status(self, goal_position, goal_quaternion):
        return False
        if(self.position == None):
            return False
        if(self.body_quat is None):
            return False

        tolerance_position = 0.2
        tolerance_orientation = 0.01

        x_diff = (not self.goal.do_x.data) or abs(self.position[0] - goal_position[0]) <= tolerance_position
        y_diff = (not self.goal.do_y.data) or abs(self.position[1] - goal_position[1]) <= tolerance_position
        z_diff = (not self.goal.do_z.data) or abs(self.position[2] - goal_position[2]) <= tolerance_position
        
        if self.goal.do_quaternion.data:
            # goal_w, goal_x, goal_y, goal_z = goal_quaternion
            # body_w, body_x, body_y, body_z = self.body_quat
            # innert_product = goal_w * body_w - goal_x * body_x - goal_y * body_y - goal_z * body_z
                
            # theta = np.arccos(2 * pow(innert_product, 2) - 1)
            quat_diff = True
            
            error = self.calculateError(self.body_quat, goal_quaternion)
            
            if abs(error.w) > tolerance_orientation:
                quat_diff = False
        
        return x_diff and y_diff and z_diff and quat_diff
        
    def get_goal_after_displace(self, goal_pose):
        goal_position = [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z]
        
        displacement_quat = np.quaternion(goal_pose.orientation.w, goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z)
        
        new_goal_quat = self.body_quat * displacement_quat
        return goal_position, new_goal_quat 
    
    def calculateError(self, q1, q2):
        q1_inv = q1.inverse()
        return q1_inv * q2
    
    
    def calculateIntegralError(self, integral_error_quat, error_quat, delta_time):
        return integral_error_quat + (error_quat * delta_time)
    
    def orthogonal_matrix(self, q):
        Qe1 = self.Q1(q)
        transpose = np.transpose(Qe1.copy())
        return np.matmul(Qe1, transpose)
        
    def controlEffort(self, goal_quat):
        if(self.body_quat is None):
            return
        delta_time = (self.time_interval[1] - self.time_interval[0])
        
        # Calculate error values
        error_quat = self.calculateError(self.body_quat, goal_quat) 
        if(error_quat.w < 0):
            error_quat = -error_quat
        
        
        proportional_effort = np.zeros(3)
        
        proportional_effort[0] = self.Kp * error_quat.x
        proportional_effort[1] = self.Kp * error_quat.y
        proportional_effort[2] = self.Kp * error_quat.z

        
        # Calculate derivative term
        derivative_effort = self.Kd * self.angular_velocity
        control_effort = proportional_effort + derivative_effort
        
        # inertial_matrix = np.array([[0.042999259180866,  0.000000000000000, -0.016440893216213],
        #                             [0.000000000000000,  0.709487776484284, 0.003794052280665], 
        #                             [-0.016440893216213, 0.003794052280665, 0.727193353794052]])
        
        inertial_matrix = np.array([[0.1376267915,  0.                , 0.                ],
                                    [0.          ,  0.6490918050833332, 0.                ], 
                                    [0.          ,  0.                , 0.6490918050833332]])
        
        torque = np.matmul(inertial_matrix, control_effort)
        self.pub_roll.publish(control_effort[0])
        self.pub_pitch.publish(control_effort[1])
        self.pub_yaw.publish(control_effort[2])     
    
        
    def control_effort_dcm(self, goal_quaternion):
        goal_rotation_matrix = quaternion.as_rotation_matrix(goal_quaternion)
        body_rotation_matrix = quaternion.as_rotation_matrix(self.body_quat)
        error_matrix = np.matmul(np.transpose(goal_rotation_matrix), body_rotation_matrix) 
        error_axis_matrix = (error_matrix - error_matrix.transpose())
        error_axis = np.array([error_axis_matrix[2, 1], error_axis_matrix[0, 2], error_axis_matrix[1, 0]])

        control_effort = self.Kp * error_axis + self.Kd * self.angular_velocity
        self.pub_roll.publish(0)
        self.pub_pitch.publish(control_effort[1])
        self.pub_yaw.publish(control_effort[2])

            