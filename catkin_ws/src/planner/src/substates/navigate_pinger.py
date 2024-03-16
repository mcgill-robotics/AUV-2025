#!/usr/bin/env python3

import rospy
import smach
from .utility.functions import *
from auv_msgs.msg import PingerBearing

# Given a pinger number (integer), navigate the AUV towards the object corresponding to that pinger number
class GoToPinger(smach.State):

    def __init__(self, control, state, mapping, pinger_num):
        super().__init__(outcomes=['success', 'failure', 'search'])
        self.control = control
        self.mapping = mapping
        self.state = state
        # An integer from 0 to 3 corresponding to a specifc pinger & object
        self.pinger_num = pinger_num

    def execute(self, ud):
        print("Starting pinger navigation. Navigating to object with pinger number", self.pinger_num) 
        
        # Get all the pinger bearings into an array to access bearings by the pinger_number
        pinger_bearings = self.state.pingerBearing
        pingers = [pinger_bearings.pinger1_bearing, pinger_bearings.pinger2_bearing, pinger_bearings.pinger3_bearing, pinger_bearings.pinger4_bearing]

        # MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        self.control.move((None, None, -2))
        self.control.rotateEuler((0,0,None))

        pinger_object = None

        # Amount of times to turn towards the pinger/move while an object is not found before giving up
        give_up_threshold = 10

        while(pinger_object is None and give_up_threshold > 0):
            print(self.state.pingerBearing)

            # The object we need to go towards will correspond to the pinger number
            pingerBearingX = pingers[self.pinger_num - 1].x
            pingerBearingY = pingers[self.pinger_num - 1].y
            pingerBearingZ = pingers[self.pinger_num - 1].z

            # Normalize (x magnitude = 1)
            bearings = np.array([pingerBearingX, pingerBearingY, pingerBearingZ])
            bearings = bearings / np.linalg.norm(bearings)

            dotProduct = np.dot(np.array([1, 0, 0]), bearings)
            
            # arctan with pinger bearing x and y to get an angle
            angle = (180/math.pi) * np.arccos(dotProduct)

            print("Angle 1", angle)

            # Add 180 degrees if the angle is negative
            if (pingerBearingX < 0 or pingerBearingY < 0):
                angle = angle + 180

            print("Angle 2", angle)

            # If the pinger is reached and no object is found (after the first attempt), exit the loop and search for it
            if angle > 180 and give_up_threshold < 10:
                return 'search'
            
            # Rotate towards the pinger bearing and move forward a bit
            self.control.rotateEuler([0, 0, angle])
            self.control.moveDeltaLocal([1, 0, 0])
            
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
            print("Object found! Centering and rotating in front of the {}".format(pinger_object[0]))
            offset_distance = -2
            rotation_amount = 180 if pinger_object[4] is None else pinger_object[4]
            dtv = degreesToVector(rotation_amount)
            offset = [] 
            for i in range(len(dtv)):
                offset.append(offset_distance * dtv[i]) 
            self.control.rotateEuler((None,None,rotation_amount))
            homing_position = (pinger_object[1] + offset[0], pinger_object[2] + offset[1], pinger_object[3])

            self.control.move(homing_position)

        print("Successfully centered in front of the {}".format(pinger_object[0]))
        print("Successfully completed pinger task!")
        
        return 'success'