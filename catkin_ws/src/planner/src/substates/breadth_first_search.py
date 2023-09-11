#!/usr/bin/env python3

import rospy
import smach
import time
import threading

#search for objects by moving in a growing square (i.e. each side of square grows in size after every rotation)
class BreadthFirstSearch(smach.State):
    def __init__(self, expansionAmt, control, mapping, target_class, min_objects, timeout, search_depth):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.detectedObject = False
        self.timeout = timeout
        self.expansionAmt = expansionAmt
        self.target_class = target_class
        self.min_objects = min_objects
        self.search_depth = search_depth

    def doBreadthFirstSearch(self):
        movement = [0,self.expansionAmt,0]
        while not rospy.is_shutdown():
            #move left
            print("Moving by {}.".format(movement))
            self.control.moveDeltaLocal(movement, face_destination=True)
            if self.detectedObject: return # stop grid search when object found
            #move left by the same amount again
            print("Moving by {}.".format(movement))
            self.control.moveDeltaLocal(movement, face_destination=True)
            if self.detectedObject: return # stop grid search when object found
            #increase distance to move by
            movement[1] += self.expansionAmt

    def execute(self, ud):
        print("Starting breadth-first search.")
        self.control.move((None, None, self.search_depth))
        self.control.rotateEuler((0,0,None))

        self.searchThread = threading.Thread(target=self.doBreadthFirstSearch)
        self.searchThread.start()
        startTime = time.time()
        while startTime + self.timeout > time.time() and not rospy.is_shutdown(): 
            if len(self.mapping.getClass(self.target_class)) >= self.min_objects:
                self.detectedObject = True
                self.searchThread.join()
                self.control.freeze_pose()
                print("Found object! Waiting 5 seconds to get more observations of object.")
                rospy.sleep(5)
                return 'success'
        self.detectedObject = True
        self.searchThread.join()
        self.control.freeze_pose()
        print("Breadth-first search timed out.")
        return 'failure'   