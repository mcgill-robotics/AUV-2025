#!/usr/bin/env python3

import rospy
import smach
import time

# State for performing a linear search for objects underwater.
class LinearSearch(smach.State):
    def __init__(self, timeout, forward_speed, auv_control, object_mapping, target_class, min_objects, search_depth):
        super().__init__(outcomes=['success', 'failure'])
        self.auv_control = auv_control
        self.object_mapping = object_mapping
        self.target_class = target_class
        self.min_objects = min_objects
        self.timeout = timeout
        self.search_depth = search_depth
        self.forward_speed = forward_speed

    def execute(self, ud):
        print("Starting linear search.")
        
        # Move to the search depth
        self.auv_control.move((None, None, self.search_depth))
        self.auv_control.rotateEuler((0, 0, None))

        # Start time for timeout
        start_time = time.time()
        
        # Perform linear search until timeout or object is found
        while start_time + self.timeout > time.time() and not rospy.is_shutdown(): 
            # Move forward
            self.auv_control.moveDeltaLocal((2, 0, 0))
            
            # Check if object is detected
            if len(self.object_mapping.getClass(self.target_class)) >= self.min_objects:
                print("Found object! Waiting 10 seconds to get more observations of object.")
                # Freeze AUV position and wait for more observations
                self.auv_control.freeze_pose()
                rospy.sleep(10)
                return 'success'
            
            # Pause briefly before next iteration
            rospy.sleep(0.1)
        
        # Freeze AUV position after timeout
        self.auv_control.freeze_pose()
        print("Linear search timed out.")
        return 'failure'
