#!/usr/bin/env python3

import rospy
import smach

from auv_msgs.msg import VisionObjectArray
from substates.utility.controller import *
from substates.utility.state import *
from substates.utility.vision import *
from substates.breadth_first_search import *

import unittest
import rostest

### Write ros testing mission for object positions on down cam ###
    
class ObjectPositionTest(unittest.TestCase):
    def test__lane_marker(self): 
        goToTarget(lane_marker_pos)
        lane_marker_map_pos = mapping.getClass(cls="Lane Marker")
        x = float(lane_marker_map_pos[0][1])
        y = float(lane_marker_map_pos[0][2])
        #z = float(lane_marker_map_pos[0][3])
        self.assertTrue(x-1 <= x <= x+1) #Checks if x value within 1 meter of expected value
        self.assertTrue(y-1 <= y <= y+1) 
        #self.assertTrue(z-1 <= z <= z+1)
    
    def test__lane_marker2(self): #Might need to nudge AUV if it gets stuck
        goToTarget(lane_marker_pos2)
        lane_marker_map_pos = mapping.getClass(cls="Lane Marker")
        x = float(lane_marker_map_pos[1][1])
        y = float(lane_marker_map_pos[1][2])
        #z = float(lane_marker_map_pos[1][3])
        self.assertTrue(x-1 <= x <= x+1) #Checks if x value within 1 meter of expected value
        self.assertTrue(y-1 <= y <= y+1) 
        #self.assertTrue(z-1 <= z <= z+1)

    def test__octagon_table(self):
        goToTarget(octagon_table_pos)
        octagon_table_map_pos = mapping.getClass(cls="Octagon Table")
        x = octagon_table_map_pos[0][1]
        y = octagon_table_map_pos[0][2]
        #z = float(octagon_table_pos[0][3])
        self.assertTrue(x-1 <= x <= x+1) #Checks if x value within 1 meter of expected value
        self.assertTrue(y-1 <= y <= y+1) 
        #self.assertTrue(z-1 <= z <= z+1)

#Helper functions
def goToTarget(target):
        print("Moving to target")
        x,y,z = target
        control.move([x,y,-2], face_destination=True)
        print("Arrived at target")

def object_map_cb(msg):
        for obj in msg.objects:
            print(obj)

if __name__ == '__main__':
    rospy.init_node('object_position_test')
    mapping = ObjectMapper()
    state = StateTracker()
    control = Controller(rospy.Time(0))
    
    #Approximate position of the Octagon Table and Lane Marker
    #Could maybe BFS instead
    octagon_table_pos=[-4.706,-9.036, 0]
    lane_marker_pos=[1.661,6.204, 0]
    lane_marker_pos2=[9.882, 1.281, 0]
    
    rostest.rosrun("planner", 'object_position_test', ObjectPositionTest) #currently in planner instead of vision
    
    