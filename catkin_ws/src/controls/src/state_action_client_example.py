#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Point
from auv_msgs.msg import StateAction, StateGoal

if __name__ == '__main__':
    print("starting test")
    rospy.init_node('example')
    position = Point()
    rotation = Point()
    position.x = 1
    position.y = 1 
    position.z = 1
    rotation.x = 1
    rotation.y = 1
    rotation.z = 1

    server = actionlib.SimpleActionClient('state_server', StateAction)
    server.wait_for_server()
    goal = StateGoal()
    goal.position = position
    goal.rotation = rotation
    server.send_goal(goal)
    #server.send_goal_and_wait(goal)
    #server.cancel_goal()