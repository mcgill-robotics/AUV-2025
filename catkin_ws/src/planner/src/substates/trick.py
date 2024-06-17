#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import String

class Trick(smach.State):
    def __init__(self, control, trick_type, num_full_spins=1):
        super().__init__(outcomes=["success", "failure"])
        self.control = control
        self.trick_type = trick_type
        self.num_full_spins = int(num_full_spins)
        self.pub_mission_display = rospy.Publisher("/mission_display", String, queue_size=1)

    def execute(self,ud):
        #STAY IN SAME POSITION AND AT FLAT ORIENTATION
        self.pub_mission_display.publish("Trick")  
        self.control.freeze_position()
        self.control.flatten()

        if self.trick_type == "roll":
            return self.execute_roll()
        elif self.trick_type == "pitch":
            return self.execute_pitch()
        elif self.trick_type == "yaw":
            return self.execute_yaw()
        #re-stabilize
        self.control.flatten()
    
    def execute_roll(self):
        print("Starting roll trick")
        for _ in range(self.num_full_spins*3): self.control.rotateDeltaEuler((120.0, 0, 0))
        print("Completed")
        return "success"   
    
    def execute_pitch(self):
        print("Starting pitch trick")
        for _ in range(self.num_full_spins*3): self.control.rotateDeltaEuler((0,120.0,0))
        print("Completed")
        return "success"  
    
    def execute_yaw(self):
        print("Starting yaw trick")
        for _ in range(self.num_full_spins*3): self.control.rotateDeltaEuler((0,0,120.0))
        print("Completed")
        return "success"