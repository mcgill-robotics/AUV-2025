#!/usr/bin/env python3

import actionlib
import smach

from auv_msgs.msg import PursueTargetAction, PursueTargetGoal
from std_msgs.msg import Float64

class DepthState(smach.State):
    def __init__(self, target_depth):
        super().__init__(outcomes=['success', 'failure'])
        self.target_depth = Float64(target_depth)
        self.client = actionlib.SimpleActionClient('pursueTarget', PursueTargetAction)

    def execute(self, ud):
        self.client.wait_for_server()
        goal = PursueTargetGoal(self.target_depth)
        self.client.send_goal(goal)
        res = self.client.wait_for_result()

        return 'success' if res else 'failure'
