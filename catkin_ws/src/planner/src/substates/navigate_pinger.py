#!/usr/bin/env python3

import rospy
import smach
from .utility.functions import *
from auv_msgs.msg import PingerBearing

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

        pinger_object = None

        give_up_threshold = 10

        # "pinger{}_bearing".format(self.pinger_num)

        while(pinger_object is None and give_up_threshold > 0):
            # we need to go towards will correspond to the pinger number
            print("Pinger bearing", self.state.pingerBearing)
            pingerBearingX = self.state.pingerBearing.pinger1_bearing.x
            pingerBearingY = self.state.pingerBearing.pinger1_bearing.y

            # Get current AUV Position when it measured the pinger bearing
            stateX = self.state.pingerBearing.state_x
            stateY = self.state.pingerBearing.state_y
            print("state X", stateX, "state Y", stateY)

            # Calculate the delta between the pinger object and the AUV in degrees
            pingerDeltaX = (180/math.pi) * (pingerBearingX - stateX)
            pingerDeltaY = (180/math.pi) * (pingerBearingY - stateY)

            print("Pinger delta X", pingerDeltaX, "Pinger delta Y", pingerDeltaY)

            # Rotate and move towards pinger position
            self.control.rotateDeltaEuler([pingerDeltaX,pingerDeltaY,0]) 
            self.control.moveDelta([1, 0, 0])
            
            # Try to get the current closest object
            # TODO: Can turn this into a specific target object if needed
            pinger_object = self.mapping.getClosestObject(pos=(self.state.x, self.state.y))

            # Keep trying if no object is found
            if pinger_object is None:
                print("No object found, moving towards pinger again.")
                give_up_threshold -= 1

        # Couldn't find the object with the specified pinger
        if pinger_object is None:
            print("No object found after", give_up_threshold, "tries. Giving up.")
            return 'failure'
        else:
            # Move towards the object
            print("Object found! Centering and rotating in front of", pinger_object[0])
            offset_distance = -2
            rotation_amount = 180 if pinger_object[4] is None else pinger_object[4]
            dtv = degreesToVector(rotation_amount)
            offset = [] 
            for i in range(len(dtv)):
                offset.append(offset_distance * dtv[i]) 
            # TODO: Check why this makes the AUV do backflips
            self.control.rotateEuler((None,None,rotation_amount))
            homing_position = (pinger_object[1] + offset[0], pinger_object[2] + offset[1], pinger_object[3])

            self.control.move(homing_position)

        # TODO: Log the type of object once that information becomes available
        print("Successfully centered in front of pinger object")
        print("Successfully completed pinger task!")
        
        return 'success'