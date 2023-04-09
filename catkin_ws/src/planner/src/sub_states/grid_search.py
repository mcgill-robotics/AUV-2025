#!/usr/bin/env python3

import rospy
import smach
from utility import *
import time
import threading

class GridSearch(smach.State):
    def __init__(self, target_class, timeout):
        super().__init__(outcomes=['success', 'failure'])
        self.target_class = target_class
        self.timeout = timeout
        self.detectedObject = False
        self.rotating = False

    def doGridSearch(self):

        def rotationComplete(): #called when rotation is complete
            self.rotating = False

        stop = (0,0,0)
        forward = (10,0,0)
        right_turn = (0,0,90)
        left_turn = (0,0,-90)
        forward_time_step = 2
        sideways_time_step = 5

        while True:
            #move forward
            print("Moving by {} for {} seconds.".format(forward, forward_time_step))
            controller.deltaVelocity(forward)
            rospy.sleep(forward_time_step)
            controller.deltaVelocity(stop)
            if self.detectedObject: return # stop grid search when object found
            #turn left 90 degrees
            print("Rotating by {}.".format(left_turn))
            self.rotating = True
            controller.rotateDelta(left_turn, rotationComplete)
            while self.rotating: rospy.sleep(0.1)
            if self.detectedObject: return # stop grid search when object found
            #move forward
            print("Moving by {} for {} seconds.".format(forward, forward_time_step))
            controller.deltaVelocity(forward)
            rospy.sleep(sideways_time_step)
            controller.deltaVelocity(stop)
            if self.detectedObject: return # stop grid search when object found
            #rotate right 90 degrees
            print("Rotating by {}.".format(right_turn))
            self.rotating = True
            controller.rotateDelta(right_turn, rotationComplete)
            while self.rotating: rospy.sleep(0.1)
            if self.detectedObject: return # stop grid search when object found
            #move forward
            print("Moving by {} for {} seconds.".format(forward, forward_time_step))
            controller.deltaVelocity(forward)
            rospy.sleep(forward_time_step)
            controller.deltaVelocity(stop)
            if self.detectedObject: return # stop grid search when object found
            #rotate right 90 degrees
            print("Rotating by {}.".format(right_turn))
            self.rotating = True
            controller.rotateDelta(right_turn, rotationComplete)
            while self.rotating: rospy.sleep(0.1)
            if self.detectedObject: return # stop grid search when object found
            #move forward
            print("Moving by {} for {} seconds.".format(forward, forward_time_step))
            controller.deltaVelocity(forward)
            rospy.sleep(sideways_time_step)
            controller.deltaVelocity(stop)
            if self.detectedObject: return # stop grid search when object found
            #turn left 90 degrees
            print("Rotating by {}.".format(left_turn))
            self.rotating = True
            controller.rotateDelta(left_turn, rotationComplete)
            while self.rotating: rospy.sleep(0.1)
            if self.detectedObject: return # stop grid search when object found
        
    def objectDetectionCb(msg):
        for i in range(len(msg.label)):
            if msg.label[i] == self.target_class:
                self.detectedObject = True
                #TODO require multiple detections before determining a success

    def execute(self, ud):
        print("Starting grid search.")

        searchThread = threading.Thread(target=self.doGridSearch)
        searchThread.start()
        startTime = time.time()

        obj_sub = rospy.Subscriber('vision/viewframe_detection', ObjectDetectionFrame, self.objectDetectCb)

        while startTime + self.timeout > time.time():
            if self.detectedObject:
                if not self.rotating: controller.preemptCurrentAction()
                searchThread.join()
                print("Found object!")
                return 'success'

        
        print("Grid search timed out.")
        return 'failure'
