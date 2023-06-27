#!/usr/bin/env python3

import rospy
import smach
from .utility.vision import *
from .utility.functions import *
import math

class NavigateLaneMarker(smach.State):
    def __init__(self, origin_class, control, state, mapping):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state
        self.origin_class = origin_class

    def execute(self, ud):
        print("Starting lane marker navigation.") 
        auv_current_position = (self.state.x, self.state.y)
        if self.origin_class == -1: # use auv current position as origin
            origin_position = auv_current_position
        else:
            origin_obj = self.mapping.getClosestObject(cls=self.origin_class, pos=(auv_current_position[0], auv_current_position[1]))
            origin_position = (origin_obj[1], origin_obj[2])
        
        lane_marker_obj = self.mapping.getClosestObject(cls=0, pos=(origin_position[0], origin_position[1]))
        if lane_marker_obj is None:
            print("No lane marker in object map! Failed.")
            return 'failure'
        
        attempts = 0
        self.control.move(lane_marker_obj[1], lane_marker_obj[2], -1)
        while lane_marker_obj[4] is None or lane_marker_obj[5] is None:
            attempts += 1
            if attempts > 5: 
                print("Lane marker angle could not be measured! Failed.")
                return 'failure'
            rospy.sleep(5.0)
            self.mapping.updateObject(lane_marker_obj)
            self.control.move(lane_marker_obj[1], lane_marker_obj[2], -1)
        
        #Waiting 10 seconds and repeating to make sure it's correct
        print("Waiting 10 seconds to ensure correct measurement of lane marker")
        self.mapping.updateObject(lane_marker_obj)
        self.control.move(lane_marker_obj[1], lane_marker_obj[2], -1)
        rospy.sleep(10)
        self.mapping.updateObject(lane_marker_obj)
        self.control.move(lane_marker_obj[1], lane_marker_obj[2], -1)

        heading1 = lane_marker_obj[4]
        heading2 = lane_marker_obj[5]

        # find heading which is pointing the least towards the AUV
        lane_marker_heading1_vec = normalize_vector(degreesToVector(heading1))
        lane_marker_heading2_vec = normalize_vector(degreesToVector(heading2))
        lane_marker_to_origin_vec = normalize_vector((lane_marker_obj[1] - origin_position[0], lane_marker_obj[2] - origin_position[1]))

        heading1_dot = dotProduct(lane_marker_to_origin_vec, lane_marker_heading1_vec)
        heading2_dot = dotProduct(lane_marker_to_origin_vec, lane_marker_heading2_vec)

        print("Rotating to lane marker target heading.")

        #   rotate to that heading
        if heading1_dot < heading2_dot: self.control.rotate((0,0,heading1))
        else: self.control.rotate((0,0,heading2))

        print("Successfully rotated to lane marker!")
        return 'success'
