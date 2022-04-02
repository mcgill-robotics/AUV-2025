#!/usr/bin/env python3

import actionlib
import smach
import rospy

from auv_msgs.msg import WaypointAction, WaypointGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import *

class DepthState(smach.State):
    def __init__(self, target_depth):
        super().__init__(outcomes=['success', 'failure'])

        target_pos = Point(x=0.0, y=0.0, z=target_depth)
        q = quaternion_from_euler(0.0, 0.0, 0.0) 
        target_orient = Quaternion(*q) 
        self.target_state = Pose(target_pos, target_orient)

        self.client = actionlib.SimpleActionClient('waypoint_server', WaypointAction)


    def execute(self, user_data):
        self.client.wait_for_server()
        goal = WaypointGoal(self.target_state)
        self.client.send_goal(goal)
        res = self.client.wait_for_result()

        return 'success' if res else 'failure'
