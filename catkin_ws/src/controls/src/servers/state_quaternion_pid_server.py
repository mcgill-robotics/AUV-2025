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
        # Calculation parameters/values
        self.goal_position = None
        self.goal_quat = None
        self.server.start()        


    def callback(self, goal):
        print("\n\nQuaternion Server got goal:\n",goal)
        self.cancelled = False
        if self.pose is not None:
            if(goal.displace.data):
                goal_position, goal_quat = self.get_goal_after_displace(goal)
                self.goal_position = goal_position
                self.goal_quat = goal_quat
            else:
                goal_position = [goal.pose.position.x, goal.pose.position.y, goal.pose.position.z]
                goal_quat = np.quaternion(goal.pose.orientation.w, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z)
                self.goal_position = goal_position
                self.goal_quat = goal_quat

            if(goal.do_x.data):
                self.pub_x_enable.publish(True)
                self.pub_x_pid.publish(goal_position[0])
                self.pub_surge.publish(0)
                self.pub_sway.publish(0)
            if(goal.do_y.data):
                self.pub_y_enable.publish(True)
                self.pub_y_pid.publish(goal_position[1])
                self.pub_surge.publish(0)
                self.pub_sway.publish(0)
            if(goal.do_z.data):
                self.pub_z_enable.publish(True)
                self.pub_z_pid.publish(goal_position[2])
                self.pub_heave.publish(0)
            if (goal.do_quaternion.data):
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
                while not self.cancelled and self.check_status(goal_position, goal_quat, goal.do_x.data, goal.do_y.data, goal.do_z.data, goal.do_quaternion.data):
                    if(rospy.get_time() - start > time_to_settle):
                        settled = True
                        print("settled")
                        break
                    rospy.sleep(0.01)

        self.server.set_succeeded()

    def get_goal_after_displace(self, goal):
        if self.goal_position is None or self.goal_quat is None:
            goal_position = [self.pose.position.x + goal.pose.position.x, self.pose.position.y + goal.pose.position.y, goal.pose.position.z + goal.pose.position.z]
            goal_quat = self.body_quat * np.quaternion(goal.pose.orientation.w, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z)
        else:
            goal_position = [self.goal_position[0] + goal.pose.position.x, self.goal_position[1] + goal.pose.position.y, self.goal_position[2] + goal.pose.position.z]
            goal_quat = self.goal_quat * np.quaternion(goal.pose.orientation.w, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z)
        
        self.goal_position = goal_position
        self.goal_quat = goal_quat

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
    