#!/usr/bin/env python3

import rospy
from substates.utility.controller import Controller
from substates.utility.state import StateTracker
import unittest
import rostest

 
## A sample python unit test
class Test1(unittest.TestCase):
    ## test 1 == 1
    def test__simple_movement(self): # only functions with 'test_'-prefix will be run!
        print(state.z)
        controls.move([0,0,-2.0])
        controls.rotate([1,0,0,0])
        # controls.move([0,0,-2])
        # controls.move([-5,-5,0])
        # controls.rotate([0.5,0.5,0.5,0.5])
        x,y,z = state.x,state.y,state.z
        qw,qx,qy,qz = state.pose.orientation.w,state.pose.orientation.x,state.pose.orientation.y,state.pose.orientation.z
        self.assertAlmostEqual(x,-5,places=0)
        self.assertAlmostEqual(y,-5,places=0)
        self.assertAlmostEqual(z,-2,places=0)
        self.assertAlmostEqual(qw,0.5,places=2)
        self.assertAlmostEqual(qx,0.5,places=2)
        self.assertAlmostEqual(qy,0.5,places=2)
        self.assertAlmostEqual(qz,0.5,places=2)

if __name__ == '__main__':
    rospy.init_node("Test1", log_level=rospy.DEBUG)
    rospy.sleep(10)
    print("hello world")
    controls = Controller(rospy.Time(0))
    state = StateTracker()
    rostest.rosrun("planner", 'Test1', Test1)
