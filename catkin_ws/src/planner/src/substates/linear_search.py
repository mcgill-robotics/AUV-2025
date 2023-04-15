#!/usr/bin/env python3

import rospy
import smach
from utility import *
import time
import threading

#ASSUMES AUV IS FACING DIRECTION TO SEARCH IN
class LinearSearch(smach.State):
    def __init__(self, timeout, forward_speed, target_classes=[], control=None):
        super().__init__(outcomes=['success', 'failure'])
        if control == None: raise ValueError("target_class argument must be a list of integers")
        self.control = control
        self.detector = vision.ObjectDetector(target_classes, callback=self.foundObject)
        self.timeout = timeout
        self.forward_speed = forward_speed

    def foundObject(self, msg):
        self.detector.stop()
        self.detectedObject = False

    def execute(self, ud):
        print("Starting linear search.")
        self.detectedObject = False
        try:
            self.control.deltaVelocityLocal((self.forward_speed, 0, 0))
            startTime = time.time()
            self.detector.start()
            while startTime + self.timeout > time.time(): 
                if self.detectedObject:
                    self.control.deltaVelocity((0,0,0))
                    print("Found object!")
                    return 'success'
            print("Linear search timed out.")
            return 'failure'
        except KeyboardInterrupt:
            self.detectedObject = True
            self.control.preemptCurrentAction()
            self.control.deltaVelocity((0,0,0))
            self.searchThread.join()
            print("Linear search interrupted by user.")
            return 'failure'

