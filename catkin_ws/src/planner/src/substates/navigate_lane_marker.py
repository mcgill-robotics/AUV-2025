#!/usr/bin/env python3

import rospy
import smach
from .utility.functions import *

class NavigateLaneMarker(smach.State):
    def __init__(self, origin_class, control, state, mapping):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state
        self.origin_class = origin_class

    def execute(self, ud):
        print("Starting lane marker navigation.") 
        
        # Move to middle of pool depth and flat orientation
        self.control.move((None, None, -2))
        self.control.rotateEuler((0, 0, None))
        
        # Determine origin position
        if self.origin_class == "":  # Use AUV current position as origin
            origin_position = (0, 0)
        else:
            origin_obj = self.mapping.getClosestObject(cls=self.origin_class, pos=(self.state.x, self.state.y))
            origin_position = (origin_obj[1], origin_obj[2])
        
        # Find the closest lane marker object
        lane_marker_obj = self.mapping.getClosestObject(cls="Lane Marker", pos=(origin_position[0], origin_position[1]))
        if lane_marker_obj is None:
            print("No lane marker in object map! Navigation failed.")
            return 'failure'
        
        attempts = 0
        while (lane_marker_obj[4] is None or lane_marker_obj[5] is None) and not rospy.is_shutdown():
            attempts += 1
            if attempts > 5: 
                print("Lane marker angle could not be measured! Navigation failed.")
                return 'failure'
            rospy.sleep(5.0)
            self.mapping.updateObject(lane_marker_obj)
            self.control.move((lane_marker_obj[1], lane_marker_obj[2], -2), face_destination=True)
        
        # Wait and repeat to ensure correct measurement of lane marker
        print("Waiting 10 seconds to ensure correct measurement of lane marker.")
        rospy.sleep(10)
        self.mapping.updateObject(lane_marker_obj)
        self.control.move((lane_marker_obj[1], lane_marker_obj[2], -2), face_destination=True)

        heading1 = lane_marker_obj[4]
        heading2 = lane_marker_obj[5]

        # Find heading that is pointing the least towards the AUV
        lane_marker_heading1_vec = normalize_vector(degreesToVector(heading1))
        lane_marker_heading2_vec = normalize_vector(degreesToVector(heading2))
        lane_marker_to_origin_vec = normalize_vector((lane_marker_obj[1] - origin_position[0], lane_marker_obj[2] - origin_position[1]))

        heading1_dot = dotProduct(lane_marker_to_origin_vec, lane_marker_heading1_vec)
        heading2_dot = dotProduct(lane_marker_to_origin_vec, lane_marker_heading2_vec)

        print("Rotating to lane marker target heading.")

        # Rotate to the calculated heading
        if heading1_dot < heading2_dot: 
            self.control.rotateEuler((0, 0, heading2))
        else: 
            self.control.rotateEuler((0, 0, heading1))

        print("Successfully rotated to lane marker!")
        return 'success'
