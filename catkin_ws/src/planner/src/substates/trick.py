#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import Float64

class Trick(smach.State):
    def __init__(self, control=None):
        super().__init__(outcomes=['success', 'failure'])
        if control == None: raise ValueError("control argument is None")
        self.control = control
    
    def execute_roll(self, ud):
        print("Starting roll trick")
        try:
            self.control.rotateDeltaEuler((120.0, 0, 0))
            self.control.rotateDeltaEuler((120.0, 0, 0))
            self.control.rotateDeltaEuler((120.0, 0, 0))

            print("Completed")
            return 'success'

        except KeyboardInterrupt:
            print("Roll trick interrupted by user.")
            return 'failure'    
    
    def execute_pitch(self, ud):
        print("Starting pitch trick")
        try:
            self.control.rotateDeltaEuler((0, 120.0, 0))
            self.control.rotateDeltaEuler((0, 120.0, 0))
            self.control.rotateDeltaEuler((0, 120.0, 0))
            print("Completed")
            return 'success'

        except KeyboardInterrupt:
            print("Pitch trick interrupted by user.")
            return 'failure'   
    
    def execute_yaw(self, ud):
        print("Starting yaw trick")
        try:
            self.control.rotateDeltaEuler((0,0,120.0))
            self.control.rotateDeltaEuler((0,0,120.0))
            self.control.rotateDeltaEuler((0,0,120.0))

            print("Completed")
            return 'success'

        except KeyboardInterrupt:
            print("Yaw trick interrupted by user.")
            return 'failure'   