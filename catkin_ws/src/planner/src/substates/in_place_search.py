#!/usr/bin/env python3

import rospy
import smach
from .utility.vision import *
import time
import threading

#search for objects by moving in a growing square (i.e. each side of square grows in size after every rotation)
class InPlaceSearch(smach.State):
        ## NOTE: target classes should be an array of elements of the form (target_class, min_objs_required)
    def __init__(self, timeout, control, mapping, target_class, min_objects):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.timeout = timeout
        self.target_class = target_class
        self.min_objects = min_objects
        self.detectedObject = False

    def doRotation(self):
        rotating = False
        def rotationComplete(arg1, arg2): #called when rotation is complete
            self.rotating = False
            
        turn_amt = (0,0,-30)

        while True:
            #move forward
            print("Rotating by {}.".format(turn_amt))
            self.rotating = True
            self.control.rotateDeltaEuler(turn_amt, rotationComplete)
            #check for object detected while rotating
            while self.rotating:
                if self.detectedObject: return # stop grid search when object found

    def execute(self, ud):
        print("Starting in-place search.")
        try:          
            self.searchThread = threading.Thread(target=self.doRotation)
            self.searchThread.start()
            print("Starting rotation.")
            startTime = time.time()
            while startTime + self.timeout > time.time(): 
                if len(self.mapping.getClass(self.target_class)) >= self.min_objects:
                    self.detectedObject = True
                    self.searchThread.join()
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

