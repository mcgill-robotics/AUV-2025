#!/usr/bin/env python3

import rospy
from substates.utility.controller import Controller
from substates.utility.state import StateTracker
import unittest
import rostest

class rotate_test(unittest.TestCase):
    def test__rotateDelta(self): # only functions with 'test_'-prefix will be run!
        controls.move([0,0,-1])
        controls.rotateDelta([0,1,0,0])
        controls.rotateDeltaEuler([None,0,180])
        controls.rotateDeltaEuler([90,None,None])
        self.assertTrue(True)
        
    def test__rotate(self):
        controls.move([0,0,-1])
        controls.rotate([0,1,0,0])
        controls.rotateEuler([0,None,180])
        controls.rotateEuler([None,90,None])
        self.assertTrue(True)

    def test__torque(self):
        controls.move([0,0,-1])
        controls.torque([10,10,0])
        rospy.sleep(1)
        controls.rotate([1,0,0,0])
        self.assertTrue(True)
        
if __name__ == '__main__':
    rospy.init_node("rotate_test")
    controls = Controller(rospy.Time(0))
    state = StateTracker()
    while state.pose is None:
        rospy.sleep(0.5)
    rostest.rosrun("planner", 'rotate_test', rotate_test)
