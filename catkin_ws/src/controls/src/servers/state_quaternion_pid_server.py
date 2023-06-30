#!/usr/bin/env python3

import rospy
from servers.base_server import BaseServer
import actionlib
from auv_msgs.msg import StateQuaternionAction
import numpy as np
import quaternion
import math
import threading

class StateQuaternionServer(BaseServer):
    def __init__(self):
        super().__init__()
        self.server = actionlib.SimpleActionServer('state_quaternion_server', StateQuaternionAction, execute_cb=self.callback, auto_start=False)
        # Calculation parameters/values
        
        self.Kp = 0.07
        self.Ki = 0
        self.Kd = -0.1
        self.integral_error_quat = np.quaternion()
        self.last_integral_time = rospy.get_time()
        self.angular_velocity = np.zeros(3)
        self.server.start()        
        self.quaternion_enabled = False
        self.pid_thread = None

    def callback(self, goal):
        print("\n\nQuaternion Server got goal:\n",goal)
        if self.pid_thread is not None and goal.do_quaternion.data:
            self.quaternion_enabled = False
            self.pid_thread.join()
        self.goal = goal
        if self.pose is not None:
            if(self.goal.displace.data):
                goal_position, goal_quat = self.get_goal_after_displace()
            else:
                goal_position = [self.goal.pose.position.x, self.goal.pose.position.y, self.goal.pose.position.z]
                goal_quat = np.quaternion(self.goal.pose.orientation.w, self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z)

            if(self.goal.do_x.data):
                self.pub_x_enable.publish(True)
                self.pub_x_pid.publish(goal_position[0])
                self.pub_surge.publish(0)
                self.pub_sway.publish(0)
            if(self.goal.do_y.data):
                self.pub_y_enable.publish(True)
                self.pub_y_pid.publish(goal_position[1])
                self.pub_surge.publish(0)
                self.pub_sway.publish(0)
            if(self.goal.do_z.data):
                self.pub_z_enable.publish(True)
                self.pub_z_pid.publish(goal_position[2])
                self.pub_heave.publish(0)
            if (self.goal.do_quaternion.data):
                self.pid_thread = threading.Thread(target=self.execute_goal, args=[goal_quat])
                self.pid_thread.start()

            time_to_settle = 4
            settled = False
            while not settled and not self.cancelled:
                start = rospy.get_time()
                while not self.cancelled and self.check_status(goal_position, goal_quat, self.goal.do_x.data, self.goal.do_y.data, self.goal.do_z.data, self.goal.do_quaternion.data):
                    if(rospy.get_time() - start > time_to_settle):
                        settled = True
                        break
                    rospy.sleep(0.01)

        self.server.set_succeeded()

    def get_goal_after_displace(self):
        goal_position = [self.pose.position.x + self.goal.pose.position.x, self.pose.position.y + self.goal.pose.position.y, self.pose.position.z + self.goal.pose.position.z]
        goal_quat = self.body_quat * np.quaternion(self.goal.pose.orientation.w, self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z)
        return goal_position, goal_quat 

    def execute_goal(self, goal_quaternion):
        self.quaternion_enabled = True
        while self.quaternion_enabled: self.controlEffort(goal_quaternion)
        
    def check_status(self, goal_position, goal_quaternion, do_x, do_y, do_z, do_quat):
        quat_error = self.calculateQuatError(self.body_quat, goal_quaternion)
        pos_x_error = self.calculatePosError(self.pose.position.x, goal_position[0])
        pos_y_error = self.calculatePosError(self.pose.position.y, goal_position[1])
        pos_z_error = self.calculatePosError(self.pose.position.z, goal_position[2])

        tolerance_position = 0.3
        tolerance_quat_w = 0.99

        if abs(quat_error.w) < tolerance_quat_w and do_quat: return False
        if abs(pos_x_error) > tolerance_position and do_x: return False
        if abs(pos_y_error) > tolerance_position and do_y: return False
        if abs(pos_z_error) > tolerance_position and do_z: return False

        return True

    def update_time_interval(self):
        self.time_interval[0] = self.time_interval[1]
        self.time_interval[1] = rospy.get_time()
    
    def calculatePosError(self, pos1, pos2):
        return abs(pos1 - pos2)

    def calculateQuatError(self, q1, q2):
        return q1.inverse() * q2
    
    def calculateIntegralError(self, error_quat, delta_time, angular_velocity):
        return [0,0,0]
    
    def orthogonal_matrix(self, q):
        Qe1 = self.Q1(q)
        transpose = np.transpose(Qe1.copy())
        return np.matmul(Qe1, transpose)
        
    def controlEffort(self, goal_quat): 
        # Calculate error values
        error_quat = self.calculateQuatError(self.body_quat, goal_quat) 
        if(error_quat.w < 0): error_quat = -error_quat
             
        proportional_effort = np.zeros(3)
        
        proportional_effort[0] = self.Kp * error_quat.x
        proportional_effort[1] = self.Kp * error_quat.y
        proportional_effort[2] = self.Kp * error_quat.z

        # Calculate derivative term
        derivative_effort = self.Kd * self.angular_velocity

        # Calculate integral term
        delta_time = rospy.get_time() - self.last_integral_time
        integral_effort = self.Ki * self.calculateIntegralError(error_quat, delta_time, self.angular_velocity)
        self.last_integral_time = rospy.get_time()

        control_effort = proportional_effort + derivative_effort # + integral_effort
        
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