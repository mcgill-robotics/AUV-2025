#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Pose, Vector3, Quaternion
import numpy as np
import quaternion

class QuaternionPID:

    def __init__(self,Kp,Ki,Kd):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.body_quat = np.quaternion(1,0,0,0)
        self.angular_velocity = np.array([0,0,0])
        self.goal_quat = None
        self.enabled = False
        self.torque_integral = np.array([0,0,0])

        self.pose_sub = rospy.Subscriber("pose",Pose,self.set_pose)
        self.angular_velocity_sub = rospy.Subscriber("angular_velocity", Vector3, self.set_ang_vel)
        self.goal_sub = rospy.Subscriber("quat_setpoint", Quaternion, self.set_goal)
        self.enable_sub = rospy.Subscriber("pid_quat_enable", Bool, self.set_enabled)

        self.pub_roll = rospy.Publisher('roll', Float64, queue_size=1)
        self.pub_pitch = rospy.Publisher('pitch', Float64, queue_size=1)
        self.pub_yaw = rospy.Publisher('yaw', Float64, queue_size=1)

    def set_pose(self,data):
        self.body_quat = np.quaternion(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
        if (self.body_quat.w < 0): self.body_quat = -self.body_quat
    
    def set_ang_vel(self, data):
        self.angular_velocity = np.array([data.x, data.y, data.z])
    
    def set_goal(self, data):
        self.goal_quat = np.quaternion(data.w, data.x, data.y, data.z)
        if (self.goal_quat.w < 0): self.goal_quat = -self.goal_quat
        self.torque_integral = np.array([0,0,0])
    
    def set_enabled(self, data):
        self.enabled = data.data

    def execute(self):
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            if self.enabled:
                roll_effort, pitch_effort, yaw_effort = self.controlEffort()
                self.pub_roll.publish(roll_effort)
                self.pub_pitch.publish(pitch_effort)
                self.pub_yaw.publish(yaw_effort)
            rate.sleep()
    
    def calculateQuatError(self, q1, q2):
        return q1.inverse() * q2

    def controlEffort(self): 
        # Calculate error values
        error_quat = self.calculateQuatError(self.body_quat, self.goal_quat) 
        if(error_quat.w < 0): error_quat = -error_quat
             
        proportional_effort = np.zeros(3)
        
        proportional_effort[0] = self.Kp * error_quat.x
        proportional_effort[1] = self.Kp * error_quat.y
        proportional_effort[2] = self.Kp * error_quat.z

        # Calculate derivative term
        derivative_effort = self.Kd * self.angular_velocity

        # Calculate integral term
        self.last_integral_time = rospy.get_time()

        control_effort = proportional_effort - derivative_effort # + integral_effort
        
        # inertial_matrix = np.array([[0.042999259180866,  0.000000000000000, -0.016440893216213],
        #                             [0.000000000000000,  0.709487776484284, 0.003794052280665], 
        #                             [-0.016440893216213, 0.003794052280665, 0.727193353794052]])
        
        # inertial_matrix = np.array([[0.1376267915,  0.                , 0.                ],
        #                             [0.          ,  0.6490918050833332, 0.                ], 
        #                             [0.          ,  0.                , 0.6490918050833332]])
        
        # torque = np.matmul(inertial_matrix, control_effort)
        return control_effort
    
if __name__ == "__main__":
    rospy.init_node("quaternion_pid", log_level=rospy.DEBUG)
    Kp = rospy.get_param("~Kp")
    Ki = rospy.get_param("~Ki")
    Kd = rospy.get_param("~Kd")
    pid = QuaternionPID(Kp, Ki, Kd)
    pid.execute()
    rospy.spin()