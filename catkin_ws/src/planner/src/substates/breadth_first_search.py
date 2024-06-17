#!/usr/bin/env python3

import rospy
import smach
import time
import threading
from std_msgs.msg import String


# search for objects by moving in a growing square (i.e. each side of square grows in size after every rotation)
class BreadthFirstSearch(smach.State):
    def __init__(self, control, mapping, target_class, min_objects):
        super().__init__(outcomes=["success", "failure"])
        self.control = control
        self.mapping = mapping
        self.detectedObject = False
        self.timeout = rospy.get_param("object_search_timeout")
        self.target_class = target_class
        self.min_objects = min_objects
        self.expansionAmt = rospy.get_param("bfs_expansion_size")
        self.pub_mission_display = rospy.Publisher(
            "/mission_display", String, queue_size=1
        )

    def doBreadthFirstSearch(self):
        movement = [0, self.expansionAmt, 0]
        while not rospy.is_shutdown():
            if self.preempt_requested():
                break
            # move left
            print("Moving by {}.".format(movement))
            self.control.moveDeltaLocal(movement, face_destination=True)
            if self.detectedObject:
                return  # stop grid search when object found
            # move left by the same amount again
            print("Moving by {}.".format(movement))
            self.control.moveDeltaLocal(movement, face_destination=True)
            if self.detectedObject:
                return  # stop grid search when object found
            # increase distance to move by
            movement[1] += self.expansionAmt

    def execute(self, ud):
        print("Starting breadth-first search.")
        self.pub_mission_display.publish("Search")
        self.control.move((None, None, rospy.get_param("nominal_depth")))
        self.control.flatten()

        self.searchThread = threading.Thread(target=self.doBreadthFirstSearch)
        self.searchThread.start()
        startTime = time.time()
        while startTime + self.timeout > time.time() and not rospy.is_shutdown():
            if self.preempt_requested():
                print("BreadthFirstSearch being preempted")
                self.service_preempt()
                return "failure"
            if len(self.mapping.getClass(self.target_class)) >= self.min_objects:
                self.detectedObject = True
                self.searchThread.join()
                self.control.freeze_pose()
                print("Found object! Waiting to get more observations of object.")
                rospy.sleep(rospy.get_param("object_observation_time"))
                return "success"
        self.detectedObject = True
        self.searchThread.join()
        self.control.freeze_pose()
        print("Breadth-first search timed out.")
        return "failure"