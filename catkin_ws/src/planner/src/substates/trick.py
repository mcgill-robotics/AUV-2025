#!/usr/bin/env python3

import rospy
import smach
import numpy as np
import math

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
        self.control.flatten()

        if self.trick_type == "roll":
            return self.execute_roll()
        elif self.trick_type == "pitch":
            return self.execute_pitch()
        elif self.trick_type == "yaw":
            return self.execute_yaw()
        #re-stabilize
        self.control.rotateEuler((0,0,None))

    def get_roll(self):
        # Extract roll angle from quaternion representation
        # Assuming the quaternion is in the form (w, x, y, z)
        q = self.orientation
        roll_rad = math.atan2(2(q.wq.x + q.yq.z), 1 - 2(q.x2 + q.y2))
        roll_deg = math.degrees(roll_rad)
        return roll_deg
    
    def execute_roll(self):
        # Get the target roll angle in radians as a parameter
        target_roll_angle = rospy.get_param('target_roll_angle')  # in radians

        print("Starting roll trick. AUV will roll until it reaches {} radians".format(target_roll_angle))

        # Move downwards a bit before rolling the AUV
        self.control.move((None, None, -1))

        # Store the initial roll angle
        initial_roll_angle = self.control.get_roll()

        # Start rolling the AUV
        self.control.torque([1, 0, 0])

        # Check the roll angle continuously until it reaches the target angle
        while True:
            current_roll_angle = self.control.get_roll()

            # Check if the absolute difference between current and initial roll angles is greater than target angle
            if abs(current_roll_angle - initial_roll_angle) >= target_roll_angle:
                self.control.torque([0, 0, 0])  # Stop rolling
                break

            rospy.sleep(0.1)  # Adjust the sleep duration as needed to control the frequency of checking

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