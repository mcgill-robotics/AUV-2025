#!/usr/bin/env python3

import rospy
import smach
from utility import *

class TestSubmergedRotations(smach.State):
    def __init__(self, hold_time):
        super().__init__(outcomes=['success', 'failure'])
        self.hold_time = hold_time

    def execute(self, ud):
        try:
            for rot in [(90,0,0), (0,90,0), (0,0,90), (-90,0,0), (0,-90,0), (0,0,-90)]:
                for i in range(4):
                    print("Rotating by {}.".format(rot))
                    controller.rotateDelta(rot)
                    print("Holding for {} seconds.".format(self.hold_time))
                    rospy.sleep(self.hold_time)
            return 'success'
        except KeyboardInterrupt:
            print("Rotation test interrupted by user.")
            return 'failure'
