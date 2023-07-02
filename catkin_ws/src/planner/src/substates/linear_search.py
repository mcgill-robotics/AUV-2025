#!/usr/bin/env python3

import rospy
import smach
from .utility.vision import *
import time
import threading

#ASSUMES AUV IS FACING DIRECTION TO SEARCH IN
class LinearSearch(smach.State):
    ## NOTE: target classes should be an array of elements of the form (target_class, min_objs_required)
    def __init__(self, timeout, forward_speed, control, mapping, target_classes):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.detectedObject = False
        self.target_classes = target_classes
        self.timeout = timeout
        self.forward_speed = forward_speed

    def execute(self, ud):
        print("Starting linear search.")
        try:
            self.control.forceLocal((self.forward_speed, 0))
            startTime = time.time()
            while startTime + self.timeout > time.time(): 
                for cls, min_objs in self.target_classes:
                    if len(self.mapping.getClass(cls)) >= min_objs:
                        self.detectedObject = True
                        self.control.stop_in_place()
                        print("Found object! Waiting 10 seconds to get more observations of object.")
                        rospy.sleep(10)
                        return 'success'
            print("Linear search timed out.")
            return 'failure'
        except KeyboardInterrupt:
            self.detectedObject = True
            self.control.stop_in_place()
            self.searchThread.join()
            print("Linear search interrupted by user.")
            return 'failure'

