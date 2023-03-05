#! /usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose
from auv_msgs.msg import StateAction, StateGoal

class StateControlActionClient():

    def __init__(self, pose):
        self.state_client = actionlib.SimpleActionClient('state_control_action', StateAction)
        print("Waiting for server")
        self.state_client.wait_for_server()
        self.send_goal(pose)
        self.state_client.wait_for_result()
        print(self.state_client.get_result())

    def send_goal(self, pose):
        goal = StateGoal()
        goal.pose = pose
        self.state_client.send_goal_and_wait(goal)


if __name__ == '__main__':
    rospy.init_node('state_action_controller')
    pose = Pose()
    pose.position.x = 0
    pose.position.y = 0 
    pose.position.z = 0
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 2
    pose.orientation.w = 0
    s = StateControlActionClient(pose)
