#!/usr/bin/env python3

import actionlib
import rospy

from auv_msgs.msg import DirectAction, DirectGoal
from smach_ros import SimpleActionState
from std_msgs.msg import Float64

class SurgeState(SimpleActionState):
    def __init__(self, effort, duration=2.0):

        # goal
        self.goal = DirectGoal()
        self.goal.type = 'DIRECT'
        self.goal.dof = 'SURGE'
        self.goal.effort = effort
        self.goal.duration = duration
        
        super().__init__('direct', DirectAction, goal=self.goal)

