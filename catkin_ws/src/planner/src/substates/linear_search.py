#!/usr/bin/env python3

import rospy
import smach
import time

#ASSUMES AUV IS FACING DIRECTION TO SEARCH IN
class LinearSearch(smach.State):
    def __init__(self, timeout, forward_speed, control, mapping, target_class, min_objects):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.detectedObject = False
        self.target_class = target_class
        self.min_objects = min_objects
        self.timeout = timeout
        self.forward_speed = forward_speed

    def execute(self, ud):
        print("Starting linear search.")
        #MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        #[COMP] set to appropriate search depth
        self.control.move((None, None, -1))
        self.control.rotateEuler((0,0,None))

        # self.control.forceLocal((self.forward_speed, 0))
        startTime = time.time()
        while startTime + self.timeout > time.time() and not rospy.is_shutdown(): 
            self.control.moveDeltaLocal((0.5,0,0))
            if len(self.mapping.getClass(self.target_class)) >= self.min_objects:
                self.detectedObject = True
                self.control.stop_in_place()
                print("Found object! Waiting 10 seconds to get more observations of object.")
                rospy.sleep(10)
                return 'success'
            rospy.sleep(0.1)
        self.control.stop_in_place()
        print("Linear search timed out.")
        return 'failure'

