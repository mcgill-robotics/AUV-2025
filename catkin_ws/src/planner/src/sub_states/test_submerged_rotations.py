#!/usr/bin/env python3

import rospy
import smach
from utility import *
import time

#assuming AUV is already at target depth

class TestSubmergedRotations(smach.State):
    def __init__(self):
        super().__init__(outcomes=['success', 'failure'])

    def execute(self, ud):
        return 'success'
