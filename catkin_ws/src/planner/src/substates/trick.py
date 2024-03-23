#!/usr/bin/env python3

import rospy
import smach
import numpy as np

class Trick(smach.State):
    def __init__(self, control, trick_type, state, num_full_spins=1):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.trick_type = trick_type
        self.state = state
        self.num_full_spins = int(num_full_spins)

    def execute(self,ud):
        #STAY IN SAME POSITION AND AT FLAT ORIENTATION
        self.control.freeze_position()
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
        # Get the number of seconds to roll the AUV as a parameter
        num_seconds_to_roll = rospy.get_param('num_seconds_to_roll')

        print("Starting roll trick. AUV will roll for {} seconds".format(num_seconds_to_roll))

        # Move downwards a bit before rolling the AUV
        self.control.move((None, None, -1))

        # Store the state that the AUV shuld return to
        before_state = [self.control.orientation.w, self.control.orientation.x, self.control.orientation.y, self.control.orientation.z]

        # Get current time to track how long the AUV has been rolling
        start = rospy.get_time()

        # Start rolling the AUV
        self.control.torque([1, 0, 0])

        # Stop rolling after num_seconds_to_roll seconds
        while True:
            if (rospy.get_time() - start > num_seconds_to_roll):
                self.control.torque([0, 0, 0])
                break

        print("Before state", before_state)
        
        # Rotate back to the original position
        self.control.rotateDelta(before_state)

        self.control.flatten()
            
        print("Completed")
        return 'success'   
    
    def execute_pitch(self):
        print("Starting pitch trick")
        for _ in range(self.num_full_spins*3): self.control.rotateDeltaEuler((0,120.0,0))
        print("Completed")
        return 'success'  
    
    def execute_yaw(self):
        print("Starting yaw trick")
        for _ in range(self.num_full_spins*3): self.control.rotateDeltaEuler((0,0,120.0))
        print("Completed")
        return 'success'