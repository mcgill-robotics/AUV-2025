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
    
    def execute_roll(self):
        # Get the target roll angle in radians as a parameter
        target_roll_angle = rospy.get_param('target_roll_angle')  # in radians

        # The number of rolls to do
        num_rolls = rospy.get_param('num_rolls')
        cur_roll = 1

        print("Starting roll trick. AUV will roll {} times until it reaches {} radians".format(num_rolls, target_roll_angle))

        # Move downwards a bit before rolling the AUV
        self.control.move((None, None, -1))

        # Store the initial roll angle
        initial_roll_angle = self.control.orientation.y + 0.00000001

        # Start rolling the AUV
        self.control.torque([1, 0, 0])

        # Check the roll angle continuously until it reaches the target angle
        while True:
            current_roll_angle = self.control.orientation.y

            print("Current", current_roll_angle, self.control.theta_y, "Target", initial_roll_angle)

            # Check if the absolute difference between current and initial roll angles is greater than target angle
            if current_roll_angle >= initial_roll_angle:
                if (cur_roll >= num_rolls):
                    self.control.torque([0, 0, 0])  # Stop rolling
                    break
                else:
                    print("Doing the next roll.")
                    cur_roll += 1

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