#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import Float64

class CoinFlip(smach.State):
    def __init__(self, control=None):
        super().__init__(outcomes=['success', 'failure'])
        if control == None: raise ValueError("control argument is None")
        self.control = control
    
    def execute(self, ud):
        print("Starting Coin Flip Mission")
        try:
            # Step 0: Set DVL to zero
            # pub_DVL = rospy.Publisher('/[DVL_something]', Float64, queue_size=1)
            # pub_DVL.publish((0, 0, 0))

            # Step 1: Rotate to 45 degrees 
            print("Rotating 45 degrees")
            self.control.rotateYaw(45)
            rospy.sleep(2)

            # Step 2: Submerge 0.5 meter
            print("Submerging 0.5 meter")
            self.control.moveDeltaLocal((0, 0, -0.5))

            # Step 2: Surge 1 meter
            print("Moving 1 meter forward")
            self.control.moveDeltaLocal((1.0, 0, 0))
            rospy.sleep(2)

            print("Stopping thrusters and floating to surface")
            self.control.kill()
            rospy.sleep(2)

            # DONE
            print("Completed")
            return 'success'

        except KeyboardInterrupt:
            print("Coin mission interrupted by used.")
            return 'failure'    