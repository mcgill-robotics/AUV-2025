#!/usr/bin/env python3

import rospy
from substates.utility.controller import Controller
from substates.utility.state import StateTracker
import unittest
import rostest


class move_test(unittest.TestCase):
    def test__move(self):  # only functions with 'test_'-prefix will be run!
        controls.move([0, 0, -2])
        controls.rotate([1, 0, 0, 0])
        controls.move([-4, -4, -1])
        controls.move([0, 0, 0])
        self.assertTrue(True)

    def test__moveDelta(self):
        controls.moveDelta([0, 0, 0])
        controls.rotate([1, 0, 0, 0])
        controls.moveDelta([-4, -4, -1])
        controls.moveDelta([0, 2, 1])
        self.assertTrue(True)

    def test__moveDeltaLocal(self):
        controls.moveDeltaLocal([0, 0, -2])
        controls.rotate([0, 0, 0, 1])
        controls.moveDeltaLocal([-4, -4, -1])
        controls.moveDeltaLocal([1, 0, 0])
        self.assertTrue(True)

    def test__force(self):
        controls.move([0, 0, -2])
        controls.rotate([1, 0, 0, 0])
        controls.forceLocal([-10, 10])
        rospy.sleep(1)
        controls.move([0, 0, 0])
        self.assertTrue(True)


if __name__ == "__main__":
    rospy.init_node("move_test")
    controls = Controller(rospy.Time(0))
    state = StateTracker()
    while state.pose is None:
        rospy.sleep(0.5)
    rospy.sleep(5)
    rostest.rosrun("planner", "move_test", move_test)
