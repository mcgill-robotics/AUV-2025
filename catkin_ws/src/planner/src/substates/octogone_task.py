#!/usr/bin/env python3

import rospy
import smach
from .utility.vision import *
import time, math


class CenterOctogone(smach.State):
    def __init__(self, control):
        super().__init__(outcomes=['success', 'failure'])
        if control == None: raise ValueError("control argument is None")
        self.control = control

    
    
    def execute(self, ud):
        '''
        Centers the octogone until error is within tolerance.
        Once the octogone is centered and scale, it returns success.
        If it loses sight of the octogone for more than timeout, returns "failure".
        Assumes the octogone is entirely in view (i.e. AUV is far enough to see the whole gate)
        '''
        print("Starting centering of octogone.")
        global last_object_detection

        timeout = 5
        targetCenterX = 0.5
        targetCenterY = 0.5
        centering_tolerance = 0.1
        centered = False
        y_increment = 1
        z_increment = 1

        startTime = time.time()

        while not (centered):
            if len(last_object_detection) > 0:
                startTime = time.time()
                center_x = last_object_detection[0]
                center_y = last_object_detection[1]
                last_object_detection = []

                print("Octogone in view at: x:{}, y:{}!".format(center_x, center_y))
                
                centering_error = math.sqrt((center_x - targetCenterX)**2 + (center_y - targetCenterY)**2)

                if centering_error > centering_tolerance: #isn't centered
                    delta_y = (targetCenterX - center_x)*y_increment
                    delta_z = (targetCenterY - center_y)*z_increment
                    print("Swaying {}".format(delta_y))
                    print("Heaving {}".format(delta_z))
                    self.control.moveDeltaLocal((0,delta_y,delta_z))
                    centered = False
                else:
                    centered = True
            else:
                print("No detection.... Timeout in " + str((startTime + timeout) - time.time()), end='\r')
                if time.time() > startTime + timeout:
                    print("Octogone no longer visible.")
                    return 'failure'
                    
        print("Successfully centered octogone!")
        return 'success'

        
class goThroughOctogone(smach.State):
    def __init__(self, control, state):
        super().__init__(outcomes=['success'])
        self.control = control
        self.state = state
        
        
    def execute(self, ud):
        '''
        Moves the AUV to the surface (ie, z=0)
        '''
        

        print("Starting movement through octogone.")
        curr_x, curr_y, _ = self.state.getPosition()
        self.control.move((curr_x, curr_y, 0))

        return 'succes'

