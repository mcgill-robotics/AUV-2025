#!/usr/bin/env python3

import rospy
import smach
import time
import threading

class InPlaceSearch(smach.State):
    def __init__(self, timeout, control, mapping, target_class, min_objects, search_depth):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.timeout = timeout
        self.target_class = target_class
        self.min_objects = min_objects
        self.detected_object = False
        self.search_depth = search_depth

    def do_rotation(self):
        turn_angle = (0, 0, -30)
        num_turns = 0
        num_full_turns = 0

        while not rospy.is_shutdown():
            if num_turns >= 360 / abs(turn_angle[2]):
                num_turns = 0
                num_full_turns += 1
                if num_full_turns == 1:
                    self.control.move((None, None, self.search_depth + 1))
                elif num_full_turns == 2:
                    self.control.move((None, None, self.search_depth - 1))
                else:
                    return
            # Rotate by the specified angle to move forward
            print("Rotating by {}.".format(turn_angle))
            self.control.rotateDeltaEuler(turn_angle)
            if self.detected_object:
                return  # Stop grid search when object found
            num_turns += 1

    def execute(self, ud):
        print("Starting in-place search.")
        # Move to middle of pool depth and flat orientation
        self.control.move((None, None, self.search_depth))
        self.control.rotateEuler((0, 0, None))

        # Start the rotation in a separate thread
        self.search_thread = threading.Thread(target=self.do_rotation)
        self.search_thread.start()
        print("Starting rotation.")
        
        start_time = time.time()
        while start_time + self.timeout > time.time() and not rospy.is_shutdown():
            if len(self.mapping.getClass(self.target_class)) >= self.min_objects:
                self.detected_object = True
                self.search_thread.join()  # Wait for rotation thread to finish
                self.control.freeze_pose()
                print("Found object! Waiting 10 seconds to get more observations of the object.")
                rospy.sleep(10)
                return 'success'
        
        # If timeout reached or ROS shutdown, stop the rotation
        self.detected_object = True
        self.search_thread.join()  # Wait for rotation thread to finish
        self.control.freeze_pose()
        print("In-place search timed out.")
        return 'failure'
