#!/usr/bin/env python3

import actionlib
import smach

from auv_msgs.msg import PursueTargetAction, PursueTargetGoal
from std_msgs.msg import Float64

class SubmergingState(smach.State):
    def __init__(self):
        super().__init__(outcomes=['submerging_success', 'submerging_failure'])
        self.client = actionlib.SimpleActionClient('pursueTarget', PursueTargetAction)
        print('waiting for server')
        self.client.wait_for_server()
        print('state detected')

    def execute(self, ud):
        target_depth = Float64(4.0)
        print("submerging to", target_depth)
        goal = PursueTargetGoal(target_depth)
        self.client.send_goal(goal)
        res = self.client.wait_for_result()
        print("finished submerging, result:", res)

        if res == 'fail':#TODO
            return 'submerging_failure'
        return 'submerging_success'
