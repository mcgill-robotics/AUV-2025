#!/usr/bin/env python3

import rospy
from servers.base_server import BaseServer
import actionlib
from auv_msgs.msg import StateQuaternionAction
import numpy as np
import quaternion
import math

class StateQuaternionServer(BaseServer):
    def __init__(self):
        super().__init__()
        print("making quaternion server")
        self.server = actionlib.SimpleActionServer('state_quaternion_server', StateQuaternionAction, execute_cb=self.callback, auto_start=False)
        # Calculation parameters/values
        
        self.Kp = 0.07
        self.Ki = 0
        self.Kd = -0.1
        self.integral_error_quat = np.quaternion()
        self.last_integral_time = rospy.get_time()
        self.angular_velocity = np.zeros(3)
        self.server.start()        

    def callback(self, goal):
        print("\n\nQuaternion Server got goal:\n",goal)
        self.goal = goal
        if self.pose != None:
            if(goal.displace):
                displaced_position, displace_quat = self.get_goal_after_displace()
                self.execute_goal(displaced_position, displace_quat)
            else:
                goal_position = [goal.position.x, goal.position.y, goal.position.z]
                goal_quat = np.quaternion(goal.orientation.w, goal.orientation.x, goal.orientation.y, goal.orientation.z)
                self.execute_goal(goal_position, goal_quat)
        
            # monitor when reached pose
            self.server.set_succeeded()
            
    def get_goal_after_displace(self):
        goal_position = [self.pose.position.x + self.goal.pose.position.x, self.pose.position.y + self.goal.pose.position.y, self.pose.position.z + self.goal.pose.position.z]
        displacement_quat = self.body_quat * np.quaternion(self.goal.pose.orientation.w, self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z)
        return goal_position, displacement_quat 

    def execute_goal(self, goal_position, goal_quaternion):
        self.publish_position_pids(goal_position)
        interval = 4
        settled = False
        print("waiting for settled")
        rate = rospy.Rate(100)  
        while not settled and not self.cancelled:
            start = rospy.get_time()
            self.controlEffort(goal_quaternion)
            while not self.cancelled and self.check_status(goal_position, goal_quaternion):
                self.controlEffort(goal_quaternion)
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
        if(self.pose.position is None): return False
        if(self.body_quat is None): return False

        quat_error = self.calculateQuatError(self.body_quat, goal_quaternion)
        pos_error = self.calculatePosError(self.pose.position, goal_position)
        tolerance_position = 0.5
        tolerance_quat_w = 0.98

        if abs(quat_error.w) > tolerance_quat_w and abs(pos_error) < tolerance_position: return True

        return False

    def cancel(self):
        self.controlEffort(self.body_quat)
        super().cancel()

    def update_time_interval(self):
        self.time_interval[0] = self.time_interval[1]
        self.time_interval[1] = rospy.get_time()
    
    def calculatePosError(self, auv_pos, goal_pos):
        return math.sqrt((auv_pos.x - goal_pos[0])**2 + (auv_pos.y - goal_pos[1])**2 + (auv_pos.z - goal_pos[2])**2)

    def calculateQuatError(self, q1, q2):
        return q1.inverse() * q2
    
    def calculateIntegralError(self, error_quat, delta_time, angular_velocity):
        return [0,0,0]
        return integral_error_quat + (error_quat * delta_time)
    
    def orthogonal_matrix(self, q):
        Qe1 = self.Q1(q)
        transpose = np.transpose(Qe1.copy())
        return np.matmul(Qe1, transpose)
        
    def controlEffort(self, goal_quat):
        if(self.body_quat is None): return
        
        # Calculate error values
        error_quat = self.calculateQuatError(self.body_quat, goal_quat) 
        if(error_quat.w < 0):
            error_quat = -error_quat
             
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

        control_effort = proportional_effort + derivative_effort + integral_effort
        
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
        self.pub_roll.publish(control_effort[0])
        self.pub_pitch.publish(control_effort[1])
        self.pub_yaw.publish(control_effort[2])

            