#!/usr/bin/env python3

import rospy
import smach

class TestSubmergedRotations(smach.State):
    def __init__(self, hold_time, control=None):
        super().__init__(outcomes=['success', 'failure'])
        if control == None: raise ValueError("target_class argument must be a list of integers")
        self.control = control
        self.hold_time = hold_time

    def execute(self, ud):
        try:
            for rot in [(90,0,0), (0,90,0), (0,0,90), (-90,0,0), (0,-90,0), (0,0,-90)]:
                for i in range(4):
                    print("Rotating by {}.".format(rot))
                    self.control.rotateDelta(rot)
                    print("Holding for {} seconds.".format(self.hold_time))
                    rospy.sleep(self.hold_time)
            return 'success'
        except KeyboardInterrupt:
            print("Rotation test interrupted by user.")
            return 'failure'