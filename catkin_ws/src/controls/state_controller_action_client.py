#! /usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose
import auv_msgs.msg

class StateControlActionClient():

    def __init__(self, depth: float):
        self.state_client = actionlib.SimpleActionClient('state_control_action', auv_msgs.msg.StateAction)
        print("Waiting for server")
        self.state_client.wait_for_server()
        self.send_goal(depth)

    def send_goal(self, depth):
        goal_pose = auv_msgs.msg.StateActionGoal()
        pose = Pose()
        pose.position.z = depth
        goal_pose.goal.setpoint = pose.position.z
        self.state_client.send_goal_and_wait(goal_pose)


if __name__ == '__main__':
    rospy.init_node('state_action_controller')
    depth = 1.0
    s = StateControlActionClient(depth)





