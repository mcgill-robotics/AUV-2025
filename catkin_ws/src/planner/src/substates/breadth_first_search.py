#!/usr/bin/env python3

import rospy
import smach
import time
import threading

class BreadthFirstSearch(smach.State):
    def __init__(self, expansion_amount, control, mapping, target_class, min_objects, timeout, search_depth):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.detected_object = False
        self.timeout = timeout
        self.expansion_amount = expansion_amount
        self.target_class = target_class
        self.min_objects = min_objects
        self.search_depth = search_depth

    def do_breadth_first_search(self):
        movement = [0, self.expansion_amount, 0]
        while not rospy.is_shutdown():
            # Move horizontally to the left
            print("Moving by {}.".format(movement))
            self.control.move_delta_local(movement, face_destination=True)
            if self.detected_object:
                return  # Stop grid search when object found
            # Move horizontally to the left by the same amount again
            print("Moving by {}.".format(movement))
            self.control.move_delta_local(movement, face_destination=True)
            if self.detected_object:
                return  # Stop grid search when object found
            # Increase distance to move horizontally by
            movement[1] += self.expansion_amount

    def execute(self, ud):
        print("Starting breadth-first search.")
        self.control.move((None, None, self.search_depth))
        self.control.rotate_euler((0, 0, None))

        self.search_thread = threading.Thread(target=self.do_breadth_first_search)
        self.search_thread.start()
        start_time = time.time()
        while start_time + self.timeout > time.time() and not rospy.is_shutdown():
            if len(self.mapping.get_class(self.target_class)) >= self.min_objects:
                self.detected_object = True
                self.search_thread.join()
                self.control.freeze_pose()
                print("Found object! Waiting 5 seconds to get more observations of the object.")
                rospy.sleep(5)
                return 'success'
        self.detected_object = True
        self.search_thread.join()
        self.control.freeze_pose()
        print("Breadth-first search timed out.")
        return 'failure'