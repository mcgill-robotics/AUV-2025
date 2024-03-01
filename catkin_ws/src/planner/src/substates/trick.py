#!/usr/bin/env python3

import rospy
import smach

class Trick(smach.State):
    def __init__(self, control, trick_type, num_full_spins=1):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.trick_type = trick_type
        self.num_full_spins = int(num_full_spins)

    def execute(self, ud):
        # STAY IN SAME POSITION AND AT FLAT ORIENTATION
        self.control.freeze_position()
        self.control.rotateEuler((0, 0, None))

        # Execute the appropriate trick based on the trick type
        if self.trick_type == "roll":
            return self.execute_roll()
        elif self.trick_type == "pitch":
            return self.execute_pitch()
        elif self.trick_type == "yaw":
            return self.execute_yaw()
        
        # If trick type is not recognized, re-stabilize
        self.control.rotateEuler((0, 0, None))
    
    def execute_roll(self):
        # Perform a roll trick
        print("Starting roll trick")
        for _ in range(self.num_full_spins * 3):  # Assuming each full spin consists of 3 rotations
            self.control.rotateDeltaEuler((120.0, 0, 0))
        print("Completed")
        return 'success'   
    
    def execute_pitch(self):
        # Perform a pitch trick
        print("Starting pitch trick")
        for _ in range(self.num_full_spins * 3):  # Assuming each full spin consists of 3 rotations
            self.control.rotateDeltaEuler((0, 120.0, 0))
        print("Completed")
        return 'success'  
    
    def execute_yaw(self):
        # Perform a yaw trick
        print("Starting yaw trick")
        for _ in range(self.num_full_spins * 3):  # Assuming each full spin consists of 3 rotations
            self.control.rotateDeltaEuler((0, 0, 120.0))
        print("Completed")
        return 'success'
