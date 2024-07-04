#!/usr/bin/env python3

import rospy
import rostest
import unittest
import cv2
import os

import sys

# Assuming test_lane_marker_measure.py is located in the tests directory
current_dir = os.path.dirname(os.path.realpath(__file__))
lane_marker_file = os.path.abspath(os.path.join(current_dir, '../../src'))
sys.path.append(lane_marker_file)
from lane_marker_measure  import measure_headings


# Naming convention for test functions:
# def test__testName(self):

class TestLaneMarkerMeasure(unittest.TestCase):
     @classmethod
     def setUpClass(cls):
          cls.final_line_angle_tolerance = rospy.get_param("final_line_angle_tolerance")
          cls.center_point_tolerance = rospy.get_param("center_point_tolerance")
          cls.images_lane_marker_folder_path = rospy.get_param("images_lane_marker_folder_path")
     
     def load_image(self, image_name):
          image_path = os.path.join(self.images_lane_marker_folder_path, image_name)
          if os.path.exists(image_path):
               image = cv2.imread(image_path, cv2.IMREAD_COLOR)
               return image
          return None

     def is_measure_correct(self, angles, angles_goal, center_point, center_point_goal):
          for i in range(len(angles)):
               if not (angles_goal[i] - self.final_line_angle_tolerance <= angles[i] <= angles_goal[i] + self.final_line_angle_tolerance):
                    return False
               if not (center_point_goal[i] - self.center_point_tolerance <= center_point[i] <= center_point_goal[i] + self.center_point_tolerance):
                    return False
          return True
     
     def run_test(self, image_name, angles_goal, center_point_goal):
        image = self.load_image(image_name)
        if image is None:
             self.fail(f"Error: {image_name} could not be loaded.")
        angles, center_point = measure_headings(image)
        if angles is None or center_point is None:
            self.fail(f"Error: measure_headings returned None for {image_name}")
        self.assertTrue(self.is_measure_correct(angles, angles_goal, center_point, center_point_goal))

     def test__0DegreesStraight(self):
        self.run_test("lane_marker_0_straight.png", [0, 0], (0, 0))

     def test__45DegreesLeft(self):
          self.run_test("lane_marker_45_left.png", [45, 0], (0, 0))

     def test__45DegreesRight(self):
          self.run_test("lane_marker_45_right.png", [-45, 0], (0, 0))

     def test__90DegreesLeft(self):
          self.run_test("lane_marker_90_left.png", [90, 0], (0, 0))

     def test__90DegreesRight(self):
          self.run_test("lane_marker_90_right.png", [-90, 0], (0, 0))


if __name__ == "__main__":
     rostest.rosrun("vision", "test_lane_marker_measure", TestLaneMarkerMeasure)