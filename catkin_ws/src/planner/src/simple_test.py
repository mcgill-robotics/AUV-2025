#!/usr/bin/env python3

import rospy
from substates.utility.controller import Controller
from substates.utility.state import StateTracker
import unittest
import rostest


## A sample python unit test
class Test1(unittest.TestCase):
    ## test 1 == 1
    def test__simple_movement(self):  # only functions with 'test_'-prefix will be run!
        controls.move([0, 0, -2])
        controls.rotate([1, 0, 0, 0])
        self.assertTrue(True)


if __name__ == "__main__":
    rospy.init_node("Test1")
    controls = Controller(rospy.Time(0))
    state = StateTracker()
    while state.pose is None:
        rospy.sleep(1)
    rospy.sleep(5)
    rostest.rosrun("planner", "settle_test", Test1)
