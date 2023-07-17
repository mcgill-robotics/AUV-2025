#!/usr/bin/env python3

import rospy
from servers.base_server import BaseServer
import actionlib
from auv_msgs.msg import StateQuaternionAction
from geometry_msgs.msg import Quaternion
import numpy as np
import quaternion

class StateQuaternionServer(BaseServer):
    def __init__(self):
        super().__init__()
        self.server = actionlib.SimpleActionServer('state_quaternion_server', StateQuaternionAction, execute_cb=self.callback, auto_start=False)
        self.previous_goal_quat = None
        self.previous_goal_x = None
        self.previous_goal_y = None
        self.previous_goal_z = None
        self.min_safe_goal_depth = -3 # [COMP] make safety values appropriate for comp pool
        self.max_safe_goal_depth = -1
        
        self.server.start()        

    def callback(self, goal):
        print("\n\nQuaternion Server got goal:\n",goal)
        self.cancelled = False
        self.goal = goal
        if self.pose is not None:
            if(self.goal.displace.data):
                goal_position, goal_quat = self.get_goal_after_displace()
            else:
                goal_position = [self.goal.pose.position.x, self.goal.pose.position.y, self.goal.pose.position.z]
                goal_quat = np.quaternion(self.goal.pose.orientation.w, self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z)

            if(self.goal.do_x.data):
                self.previous_goal_x = goal_position[0]
                self.pub_x_enable.publish(True)
                self.pub_x_pid.publish(goal_position[0])
                self.pub_surge.publish(0)
                self.pub_sway.publish(0)
            if(self.goal.do_y.data):
                self.previous_goal_y = goal_position[1]
                self.pub_y_enable.publish(True)
                self.pub_y_pid.publish(goal_position[1])
                self.pub_surge.publish(0)
                self.pub_sway.publish(0)
            if(self.goal.do_z.data):
                self.previous_goal_z = goal_position[2]
                self.pub_z_enable.publish(True)
                safe_goal = max(min(goal_position[2], self.max_safe_goal_depth), self.min_safe_goal_depth)
                if (safe_goal != goal_position[2]): print("WARN: Goal changed from {}m to {}m for safety.".format(goal_position[2], safe_goal))
                self.pub_z_pid.publish(safe_goal)
                self.pub_heave.publish(0)
            if (self.goal.do_quaternion.data):
                self.previous_goal_quat = goal_quat
                self.pub_quat_enable.publish(True)
                goal_msg = Quaternion()
                goal_msg.w = goal_quat.w
                goal_msg.x = goal_quat.x
                goal_msg.y = goal_quat.y
                goal_msg.z = goal_quat.z
                self.pub_quat_pid.publish(goal_msg)

            time_to_settle = 4
            settled = False
            while not settled and not self.cancelled:
                start = rospy.get_time()
                while not self.cancelled and self.check_status(goal_position, goal_quat, self.goal.do_x.data, self.goal.do_y.data, self.goal.do_z.data, self.goal.do_quaternion.data):
                    if(rospy.get_time() - start > time_to_settle):
                        settled = True
                        print("settled")
                        break
                    rospy.sleep(0.01)

        self.server.set_succeeded()

    def get_goal_after_displace(self):
        if self.previous_goal_x is None: self.previous_goal_x = self.pose.position.x
        if self.previous_goal_y is None: self.previous_goal_y = self.pose.position.y
        if self.previous_goal_z is None: self.previous_goal_z = self.pose.position.z
        goal_position = [self.previous_goal_x + self.goal.pose.position.x, self.previous_goal_y + self.goal.pose.position.y, self.previous_goal_z + self.goal.pose.position.z]
        if self.previous_goal_quat is None: self.previous_goal_quat = self.body_quat
        goal_quat = self.previous_goal_quat * np.quaternion(self.goal.pose.orientation.w, self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z)
        return goal_position, goal_quat 
        
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

    def calculatePosError(self, pos1, pos2):
        return abs(pos1 - pos2)

    def calculateQuatError(self, q1, q2):
        return q1.inverse() * q2
    
