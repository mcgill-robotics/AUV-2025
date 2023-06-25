#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import Float64

class QuaternionTest(smach.State):
    def __init__(self, control=None):
        super().__init__(outcomes=['success', 'failure'])
        if control == None: raise ValueError("control argument is None")
        self.control = control
    
    def execute(self, ud):
        print("Starting Quaternion Mission")
        try:
            # self.control.move((0,0,0))
            position = [0, 0, 0]
            quaternion = [1, 0, 0, 0]
            self.control.quaternion_action(position, quaternion)
            # DONE
            print("Completed")
            return 'success'

        except KeyboardInterrupt:
            print("Quaternion mission interrupted by used.")
            return 'failure'    

    
