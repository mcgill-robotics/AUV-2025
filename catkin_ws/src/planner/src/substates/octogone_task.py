#!/usr/bin/env python3

import rospy
import smach
from .utility.vision import *



class NavigateOctogone(smach.State):
    def __init__(self, control, state, mapping):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state

    def execute(self, ud):
        print("Starting octogone navigation.") 
        auv_current_position = self.state.getPosition()
        # use auv current position as origin
        origin_position = auv_current_position
       
        
        octogone_obj = self.mapping.getClosestObject(3, (origin_position[0], origin_position[1]))
        
        if octogone_obj is None:
            print("No octogone in object map! Failed.")
            return 'failure'
        

        print("Moving to the center of the octogone and to the surface ")
        self.control.move((octogone_obj[1], octogone_obj[2]), -1)
        self.control.kill()
        


        print("Successfully navigated through the octogone !")
        return 'success'