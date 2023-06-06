#!/usr/bin/env python3

import rospy
import smach

class RepositionGateBuoy(smach.State):
    def __init__(self, control):
        super().__init__(outcomes=['success', 'failure'])
        if control == None: raise ValueError("control argument is None")
        self.control = control
    
    def execute(self, ud):
        """ Rotate 90 degrees """
        print("Start reposition for Gate/Buoy")
        try:
            self.control.rotateYaw(45)
            print("Completed")
            return 'success'

        except KeyboardInterrupt:
            print("Reposition gate/buoy interrupted by user.")
            return 'failure' 

class RepositionLaneMarker(smach.State):
    def __init__(self, control):
        super().__init__(outcomes=['success', 'failure'])
        if control == None: raise ValueError("control argument is None")
        self.control = control
    
    def execute(self, ud):
        """ Move up by 0.5 meter """
        print("Start reposition for Lane Marker")
        try:
            self.control.moveDeltaLocal((0, 0, 0.5))
            print("Completed")
            return 'success'

        except KeyboardInterrupt:
            print("Reposition lane marker interrupted by user.")
            return 'failure' 
        

