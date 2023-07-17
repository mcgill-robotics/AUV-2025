#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import Float64

class Trick(smach.State):
    def __init__(self, control, trick_type, num_full_spins=1):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.trick_type = trick_type
        self.num_full_spins = int(num_full_spins)

    def execute(self,ud):
        #STAY IN SAME POSITION AND AT FLAT ORIENTATION
        self.control.moveDelta((0,0,0), callback=lambda a,b: None)
        self.control.rotateEuler((0,0,None))

        if self.trick_type == "roll":
            return self.execute_roll()
        elif self.trick_type == "pitch":
            return self.execute_pitch()
        elif self.trick_type == "yaw":
            return self.execute_yaw()
        #re-stabilize
        self.control.rotateEuler((0,0,None))
    
    def execute_roll(self):
        print("Starting roll trick")
        for _ in range(self.num_full_spins*3): self.control.rotateEulerDelta((120.0, 0, 0))
        print("Completed")
        return 'success'   
    
    def execute_pitch(self):
        print("Starting pitch trick")
        for _ in range(self.num_full_spins*3): self.control.rotateEulerDelta((0,120.0,0))
        print("Completed")
        return 'success'  
    
    def execute_yaw(self):
        print("Starting yaw trick")
        for _ in range(self.num_full_spins*3): self.control.rotateEulerDelta((0,0,120.0))
        print("Completed")
        return 'success'