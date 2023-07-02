#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import Float64

class QuaternionTest(smach.State):
    def __init__(self, control):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
    
    def execute(self, ud):
        print("Starting Quaternion Mission")
        try:
            position = [0, 0, 0]
            quaternion = [0.7071068, 0, 0.7071068, 0]
            self.control.quaternion_action(position, quaternion)
            # DONE
            print("Completed")
            return 'success'

        except KeyboardInterrupt:
            print("Quaternion mission interrupted by used.")
            return 'failure'    

    
