#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import Float64

class CoinFlip(smach.State):
    def __init__(self, control=None):
        super().__init__(outcomes=['success', 'failure'])
        if control == None: raise ValueError("control argument is None")
        self.control = control
    
    def execute_roll(self, ud):
        print("Starting roll trick")
        try:
            self.control.rotateDelta((360.0, 0, 0))

            print("Completed")
            return 'success'

        except KeyboardInterrupt:
            print("Coin mission interrupted by used.")
            return 'failure'    
    
    def execute_pitch(self, ud):
        print("Starting pitch trick")
        try:
            self.control.rotateYaw(45)
            rospy.sleep(2)


            # DONE
            print("Completed")
            return 'success'

        except KeyboardInterrupt:
            print("Coin mission interrupted by used.")
            return 'failure'   
    
    def execute_yaw(self, ud):
        print("Starting yaw trick")
        try:
            # Step 1: Rotate to 45 degrees 
            print("Rotating 45 degrees")
            self.control.rotateYaw(45)
            rospy.sleep(2)


            # DONE
            print("Completed")
            return 'success'

        except KeyboardInterrupt:
            print("Coin mission interrupted by used.")
            return 'failure'   