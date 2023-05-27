#!/usr/bin/env python3

import rospy
import smach
from .utility.vision import *
import time
import threading

#search for objects by moving in a growing square (i.e. each side of square grows in size after every rotation)
class BreadthFirstSearch(smach.State):
    def __init__(self, timeout, expansionAmt, target_classes=[], control=None):
        super().__init__(outcomes=['success', 'failure'])
        if control == None: raise ValueError("control argument is None")
        self.control = control
        self.detector = ObjectDetector(target_classes, callback=self.foundObject)
        self.timeout = timeout
        self.expansionAmt = expansionAmt

    def doBreadthFirstSearch(self):
        rotating = False
        moving = False
        def rotationComplete(): #called when rotation is complete
            global rotating
            rotating = False
        def movementComplete(): #called when translation is complete
            global moving
            moving = False
            
        movement = [1,0,0]
        right_turn = (0,0,-90)

        while True:
            #move forward
            print("Moving by {}.".format(movement))
            moving = True
            self.control.moveDeltaLocal(movement, movementComplete)
            #check for object detected while moving
            while moving:
                if self.detectedObject: return # stop grid search when object found
            #increase distance to move forward
            movement[0] += self.expansionAmt
            #rotate right 90 degrees
            print("Rotating by {}.".format(right_turn))
            rotating = True
            self.control.rotateDelta(right_turn, rotationComplete)
            #check for object detected while rotating
            while rotating:
                if self.detectedObject: return # stop grid search when object found
    
    def foundObject(self, msg):
        self.detector.stop()
        self.detectedObject = True

    def execute(self, ud):
        print("Starting breadth-first search.")
        self.detectedObject = False

        try:
            self.searchThread = threading.Thread(target=self.doBreadthFirstSearch)
            self.searchThread.start()
            startTime = time.time()
            self.detector.start()
            while startTime + self.timeout > time.time(): 
                if self.detectedObject:         
                    self.control.stop_in_place()
                    self.searchThread.join()
                    print("Found object!")
                    return 'success'
            print("Breadth-first search timed out.")
            return 'failure'
        except KeyboardInterrupt:
            self.detectedObject = True
            self.control.stop_in_place()
            self.searchThread.join()
            print("Breadth-first search interrupted by user.")
            return 'failure'

