#!/usr/bin/env python3

import rospy
import smach
from .utility.vision import *
import math

class NavigateLaneMarker(smach.State):
    def __init__(self, control, state, mapping):
        super().__init__(outcomes=['success'])
        self.control = control
        self.mapping = mapping
        self.state = state
        
    def degreesToVector(self, angleDegrees):
        angleRadians = (angleDegrees - 90) * math.pi / 18
        x = math.cos(angleRadians)
        y = math.sin(angleRadians)
        return [x, y]

    def normalize_vector(self, vector2D):
        magnitude = math.sqrt(vector2D[0] ** 2 + vector2D[1] ** 2)
        if magnitude == 0:
            return vector2D
        normalized_x = vector2D[0] / magnitude
        normalized_y = vector2D[1] / magnitude
        return [normalized_x, normalized_y]

    def dotProduct(self, v1, v2):
        return v1[0]*v2[0] + v1[1]*v2[1]

    def execute(self, ud):
        print("Starting lane marker navigation.") 
        lane_marker_obj = self.mapping.getClosestObject(0)
        heading1 = lane_marker_obj[4]
        heading2 = lane_marker_obj[5]
        auv_current_position = self.state.getPosition()

        # find heading which is pointing the least towards the AUV
        lane_marker_heading1_vec = self.normalizeVector(self.degreesToVector(heading1))
        lane_marker_heading2_vec = self.normalizeVector(self.degreesToVector(heading2))
        lane_marker_to_auv_vec = self.normalizeVector((lane_marker_obj[0] - auv_current_position[0], lane_marker_obj[1] - auv_current_position[1]))

        heading1_dot = self.dotProduct(lane_marker_to_auv_vec, lane_marker_heading1_vec)
        heading2_dot = self.dotProduct(lane_marker_to_auv_vec, lane_marker_heading2_vec)

        #   rotate to that heading
        if heading1_dot < heading2_dot: self.control.rotate((0,0,heading1))
        else: self.control.rotate((0,0,heading2))

        print("Successfully rotated to lane marker!")
        return 'success'
