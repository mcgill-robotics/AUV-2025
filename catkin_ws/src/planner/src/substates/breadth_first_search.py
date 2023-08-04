#!/usr/bin/env python3

import rospy
import smach
import time
import threading

#search for objects by moving in a growing square (i.e. each side of square grows in size after every rotation)
class BreadthFirstSearch(smach.State):
    def __init__(self, expansionAmt, control, mapping, target_class, min_objects, timeout):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.detectedObject = False
        self.timeout = timeout
        self.expansionAmt = expansionAmt
        self.target_class = target_class
        self.min_objects = min_objects

    def doBreadthFirstSearch(self):
        global moving
        moving = False                
            
        movement = [0,1,0]

        while not rospy.is_shutdown():
            #move left
            print("Moving by {}.".format(movement))
            moving = True
            # [TODO] FIX
            self.control.moveDeltaLocal(movement, callback=movementComplete, face_destination=True)
            #check for object detected while moving
            while moving and not rospy.is_shutdown():
                if self.detectedObject: return # stop grid search when object found
            #move left by the same amount again
            print("Moving by {}.".format(movement))
            moving = True
            self.control.moveDeltaLocal(movement, callback=movementComplete, face_destination=True)
            #check for object detected while moving
            while moving and not rospy.is_shutdown():
                if self.detectedObject: return # stop grid search when object found
            #increase distance to move by
            movement[1] += self.expansionAmt

    def execute(self, ud):
        print("Starting breadth-first search.")
        #MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        # [COMP] make sure depth is appropriate here
        self.control.move((None, None, -0.5))
        self.control.rotateEuler((0,0,None))

        self.searchThread = threading.Thread(target=self.doBreadthFirstSearch)
        self.searchThread.start()
        startTime = time.time()
        while startTime + self.timeout > time.time() and not rospy.is_shutdown(): 
            if len(self.mapping.getClass(self.target_class)) >= self.min_objects:
                self.detectedObject = True
                self.searchThread.join()
                self.control.stop_in_place()
                print("Found object! Waiting 10 seconds to get more observations of object.")
                rospy.sleep(10)
                return 'success'
        self.control.stop_in_place()
        print("Breadth-first search timed out.")
        return 'failure'

def movementComplete(msg1=None, msg2=None): #called when translation is complete
    global moving
    moving = False    