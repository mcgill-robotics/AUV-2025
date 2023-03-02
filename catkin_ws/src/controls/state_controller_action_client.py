#! /usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose
from auv_msgs.msg import StateAction

class StateControlActionClient():

    def __init__(self, pose):
        self.state_client = actionlib.SimpleActionClient('state_control_action', StateAction)
        print("Waiting for server")
        self.state_client.wait_for_server()
        self.send_goal(pose)

    def send_goal(self, pose):
        action = StateAction()
        action.pose = pose
        goal_pose = auv_msgs.msg.StateActionGoal()
        self.state_client.send_goal_and_wait(action)


if __name__ == '__main__':
    rospy.init_node('state_action_controller')
    pose = Pose()
    pose.position.x = 1
    pose.position.y = 2 
    pose.position.z = 3
    pose.orientation.x = 4
    pose.orientation.y = 5
    pose.orientation.z = 6
    pose.orientation.w = 7
    s = StateControlActionClient(pose)
