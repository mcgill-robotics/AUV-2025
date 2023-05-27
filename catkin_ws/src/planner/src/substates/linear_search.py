#!/usr/bin/env python3

import rospy
import smach
from .utility.vision import *
import time
import threading

#ASSUMES AUV IS FACING DIRECTION TO SEARCH IN
class LinearSearch(smach.State):
    def __init__(self, timeout, forward_speed, target_classes=[], control=None):
        super().__init__(outcomes=['success', 'failure'])
        if control == None: raise ValueError("control argument is None")
        self.control = control
        self.detector = ObjectDetector(target_classes, callback=self.foundObject)
        self.timeout = timeout
        self.forward_speed = forward_speed

    def foundObject(self, msg):
        self.detector.stop()
        self.detectedObject = False

    def execute(self, ud):
        print("Starting linear search.")
        self.detectedObject = False
        try:
            self.control.velocityLocal((self.forward_speed, 0, 0))
            startTime = time.time()
            self.detector.start()
            while startTime + self.timeout > time.time(): 
                if self.detectedObject:
                    self.control.stop()
                    print("Found object!")
                    return 'success'
            print("Linear search timed out.")
            return 'failure'
        except KeyboardInterrupt:
            self.detectedObject = True
            self.control.stop()
            self.searchThread.join()
            print("Linear search interrupted by user.")
            return 'failure'

