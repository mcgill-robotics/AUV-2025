#!/usr/bin/env python3

import rospy
import smach
from utility import *
import time
import threading

class GridSearch(smach.State):
    def __init__(self, target_class, timeout, min_consecutive_detections):
        super().__init__(outcomes=['success', 'failure'])
        self.target_class = target_class
        self.timeout = timeout
        self.detectedObject = False
        self.rotating = False
        self.moving = False
        self.consecutive_detections = 0
        self.min_consecutive_detections = min_consecutive_detections

    def doGridSearch(self):

        def rotationComplete(): #called when rotation is complete
            self.rotating = False

        def movementComplete(): #called when rotation is complete
            self.moving = False

        stop = (0,0,0)
        forward_movement = (1,0,0)
        side_movement = (2,0,0)
        right_turn = (0,0,90)
        left_turn = (0,0,-90)

        while True:
            #move forward
            print("Moving by {}.".format(forward_movement))
            self.moving = True
            controller.moveDeltaLocal(forward_movement, movementComplete)
            while self.moving: if self.detectedObject: return # stop grid search when object found
            #turn left 90 degrees
            print("Rotating by {}.".format(left_turn))
            self.rotating = True
            controller.rotateDelta(left_turn, rotationComplete)
            while self.rotating: if self.detectedObject: return # stop grid search when object found
            #move forward
            print("Moving by {}.".format(side_movement))
            self.moving = True
            controller.moveDeltaLocal(side_movement, movementComplete)
            while self.moving: if self.detectedObject: return # stop grid search when object found
            #rotate right 90 degrees
            print("Rotating by {}.".format(right_turn))
            self.rotating = True
            controller.rotateDelta(right_turn, rotationComplete)
            while self.rotating: if self.detectedObject: return # stop grid search when object found
            #move forward
            print("Moving by {}.".format(forward_movement))
            self.moving = True
            controller.moveDeltaLocal(forward_movement, movementComplete)
            while self.moving: if self.detectedObject: return # stop grid search when object found
            #rotate right 90 degrees
            print("Rotating by {}.".format(right_turn))
            self.rotating = True
            controller.rotateDelta(right_turn, rotationComplete)
            while self.rotating: if self.detectedObject: return # stop grid search when object found
            #move forward
            print("Moving by {}.".format(side_movement))
            self.moving = True
            controller.moveDeltaLocal(side_movement, movementComplete)
            while self.moving: if self.detectedObject: return # stop grid search when object found
            #turn left 90 degrees
            print("Rotating by {}.".format(left_turn))
            self.rotating = True
            controller.rotateDelta(left_turn, rotationComplete)
            while self.rotating: if self.detectedObject: return # stop grid search when object found
    
    def objectDetectionCb(msg):
        for i in range(len(msg.label)):
            if msg.label[i] == self.target_class:
                self.consecutive_detections += 1
                if self.consecutive_detections >= self.min_consecutive_detections: self.detectedObject = True
                return
                #TODO require multiple detections before determining a success
        self.consecutive_detections = 0

    def execute(self, ud):
        print("Starting grid search.")

        searchThread = threading.Thread(target=self.doGridSearch)
        searchThread.start()
        startTime = time.time()

        obj_sub = rospy.Subscriber('vision/viewframe_detection', ObjectDetectionFrame, self.objectDetectCb)
        try:
            while startTime + self.timeout > time.time():
                if self.detectedObject:
                    controller.preemptCurrentAction()
                    controller.deltaVelocity((0,0,0))
                    searchThread.join()
                    print("Found object!")
                    return 'success'
            
            print("Grid search timed out.")
            return 'failure'
        except KeyboardInterrupt:
            self.detectedObject = True
            controller.preemptCurrentAction()
            controller.deltaVelocity((0,0,0))
            searchThread.join()
            print("Grid search interrupted by user.")
            return 'failure'

