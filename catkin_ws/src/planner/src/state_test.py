#!/usr/bin/env python3

import rospy
from substates.utility.controller import Controller
from substates.utility.state import StateTracker
import unittest
import rostest

class state_test(unittest.TestCase):
    def test__state(self): # only functions with 'test_'-prefix will be run!
        controls.state([0,0,-2], [1,0,0,0])
        controls.state([None,None,None], [1,0,0,0])
        controls.stateEuler([None,0,-1], [None,0,0])
        controls.stateEuler([2,None,None], [90, None, None])
        self.assertTrue(True)
    
    def test__stateDelta(self): # only functions with 'test_'-prefix will be run!
        controls.stateDelta([None,None,-2], [1,0,0,0])
        controls.stateDelta([1,1,None], [0,1,0,0])
        controls.stateDeltaEuler([None,0,-1], [None,45,0])
        controls.stateDeltaEuler([2,None,None], [90, None, None])
        self.assertTrue(True)

if __name__ == '__main__':
    rospy.init_node("state_test")
    controls = Controller(rospy.Time(0))
    state = StateTracker()
    while state.pose is None:
        rospy.sleep(0.5)
    rostest.rosrun("planner", 'state_test', state_test)
