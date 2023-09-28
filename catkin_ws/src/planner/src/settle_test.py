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
        controls.move([0,0,-2])
        controls.rotate([1,0,0,0])
        rospy.sleep(90)
        x,y,z = state.x,state.y,state.z
        qw,qx,qy,qz = state.pose.orientation.w,state.pose.orientation.x,state.pose.orientation.y,state.pose.orientation.z
        if qw < 0:
            qw = -qw
            qx = -qx
            qy = -qy
            qz = -qz
        self.assertAlmostEqual(x,0,places=0)
        self.assertAlmostEqual(y,0,places=0)
        self.assertAlmostEqual(z,-2,places=0)
        self.assertAlmostEqual(qw,1,places=1)
        self.assertAlmostEqual(qx,0,places=1)
        self.assertAlmostEqual(qy,0,places=1)
        self.assertAlmostEqual(qz,0,places=1)

if __name__ == '__main__':
    rospy.init_node("Test1")
    controls = Controller(rospy.Time(0))
    state = StateTracker()
    rospy.sleep(60)
    rostest.rosrun("planner", 'settle_test', Test1)
