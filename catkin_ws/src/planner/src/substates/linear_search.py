#!/usr/bin/env python3

import rospy
import smach
import time

#ASSUMES AUV IS FACING DIRECTION TO SEARCH IN
class LinearSearch(smach.State):
    def __init__(self, control, mapping, target_class, min_objects):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.detectedObject = False
        self.target_class = target_class
        self.min_objects = min_objects
        self.timeout = rospy.get_param("object_search_timeout")

    def execute(self, ud):
        print("Starting linear search.")
        self.control.move((None, None, rospy.get_param("nominal_depth")))
        self.control.flatten()

        startTime = time.time()
        while startTime + self.timeout > time.time() and not rospy.is_shutdown(): 
            self.control.moveDeltaLocal((rospy.get_param("linear_search_step_size"),0,0))
            if len(self.mapping.getClass(self.target_class)) >= self.min_objects:
                self.detectedObject = True
                self.control.freeze_pose()
                print("Found object! Waiting 10 seconds to get more observations of object.")
                rospy.sleep(rospy.get_param("object_observation_time"))
                return 'success'
            rospy.sleep(0.1)
        self.control.freeze_pose()
        print("Linear search timed out.")
        return 'failure'
