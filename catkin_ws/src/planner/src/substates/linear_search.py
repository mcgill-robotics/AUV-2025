#!/usr/bin/env python3

import rospy
import smach
import time

#ASSUMES AUV IS FACING DIRECTION TO SEARCH IN
class LinearSearch(smach.State):
    def __init__(self, timeout, forward_speed, control, mapping, target_class, min_objects, search_depth):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.detectedObject = False
        self.target_class = target_class
        self.min_objects = min_objects
        self.timeout = timeout
        self.search_depth = search_depth
        self.forward_speed = forward_speed

    def execute(self, ud):
        print("Starting linear search.")
        self.control.move((None, None, self.search_depth))
        self.control.rotateEuler((0,0,None))

        startTime = time.time()
        while startTime + self.timeout > time.time() and not rospy.is_shutdown(): 
            self.control.moveDeltaLocal((2,0,0))
            if len(self.mapping.getClass(self.target_class)) >= self.min_objects:
                self.detectedObject = True
                self.control.freeze_pose()
                print("Found object! Waiting 10 seconds to get more observations of object.")
                rospy.sleep(10)
                return 'success'
            rospy.sleep(0.1)
        self.control.freeze_pose()
        print("Linear search timed out.")
        return 'failure'

