#!/usr/bin/env python3

import rospy
import smach
from .utility.vision import *
import time
import threading

#search for objects by moving in a growing square (i.e. each side of square grows in size after every rotation)
class InPlaceSearch(smach.State):
        ## NOTE: target classes should be an array of elements of the form (target_class, min_objs_required)
    def __init__(self, timeout, target_classes=[], min_objs=1, control=None, mapping=None):
        super().__init__(outcomes=['success', 'failure'])
        if control == None: raise ValueError("control argument is None")
        self.control = control
        if mapping == None: raise ValueError("mapping argument is None")
        self.mapping = mapping
        self.timeout = timeout
        self.target_classes = target_classes

    def execute(self, ud):
        print("Starting in-place search.")
        try:          
            turning_speed = (0,0,-3)
            print("Setting torque to {}.".format(turning_speed))
            self.control.torque(turning_speed)
            startTime = time.time()
            while startTime + self.timeout > time.time(): 
                for cls, min_objs in self.target_classes:
                    if len(self.mapping.getClass(cls)) >= min_objs:
                        self.control.stop_in_place()
                        print("Found object! Waiting 10 seconds to get more observations of object.")
                        rospy.sleep(10)
                        return 'success'
            print("In-place search timed out.")
            return 'failure'
        except KeyboardInterrupt:
            self.control.stop_in_place()
            print("In-place search interrupted by user.")
            return 'failure'

