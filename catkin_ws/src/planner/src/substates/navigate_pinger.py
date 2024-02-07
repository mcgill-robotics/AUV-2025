#!/usr/bin/env python3

import rospy
import smach
from .utility.functions import *

# Given a pinger number (integer), navigate the AUV towards the object corresponding to that pinger number
class GoToPinger(smach.State):

    def __init__(self, control, state, mapping, pinger_num):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state
        # An integer from 0 to 3 corresponding to a specifc pinger & object
        self.pinger_num = pinger_num

    def execute(self, ud):
        print("Starting pinger navigation. Navigating to object with pinger number", self.pinger_num) 

        # MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        self.control.move((None, None, -2))
        self.control.rotateEuler((0,0,None))

        # TODO [COMP]: Just going to Octagon table for now, but later the object we need to  go towards will correspond to the pinger number
        pinger_object = self.mapping.getClosestObject(cls="Octagon Table", pos=(self.state.x, self.state.y))

        # Couldn't find the object with the specified pinger
        if pinger_object is None:
            print("No object in object map! Failed.")
            return 'failure'
    
        print("Centering and rotating in front of pinger object.")
        offset_distance = -2
        dtv = degreesToVector(pinger_object[4])
        offset = [] 
        for i in range(len(dtv)):
            offset.append(offset_distance * dtv[i]) 
        homing_rotation = (0,0,pinger_object[4])
        homing_position = (pinger_object[1] + offset[0], pinger_object[2] + offset[1], pinger_object[3])

        # Rotate to the exact angle towards pinger object
        self.control.rotateEuler(homing_rotation) 
        # Move in front of pinger object
        self.control.move(homing_position) 

        # TODO: Log the type of object once that information becomes available
        print("Successfully centered in front of pinger object")

        print("Successfully completed pinger task!")
        
        return 'success'