#!/usr/bin/env python3

import rospy
import smach
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
        num_turns = 0
        num_full_turns = 0

        while not rospy.is_shutdown():
            if (num_turns >= 360/abs(turn_amt[2])):
                num_turns = 0
                num_full_turns += 1
                if num_full_turns == 1: self.control.move((None,None,-3))
                elif num_full_turns == 2: self.control.move((None,None,-1))
                else: return
            #move forward
            print("Rotating by {}.".format(turn_amt))
            self.rotating = True
            self.control.rotateDeltaEuler(turn_amt, rotationComplete)
            #check for object detected while rotating
            while self.rotating:
                if self.detectedObject: return # stop grid search when object found
            num_turns += 1

    def execute(self, ud):
        print("Starting in-place search.")
        #MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        self.control.move((None, None, -2), callback=lambda a,b: None)
        self.control.moveDelta((0,0,0), callback=lambda a,b: None)
        self.control.rotateEuler((0,0,None))

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
        self.control.stop_in_place()
        print("In-place search timed out.")
        return 'failure'
