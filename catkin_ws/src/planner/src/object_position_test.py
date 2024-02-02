#!/usr/bin/env python3

import rospy

from auv_msgs.msg import VisionObjectArray
from substates.utility.controller import Controller
from substates.utility.vision import ObjectMapper
from substates.breadth_first_search import *

# import unittest
# import rostest

### Write ros testing mission for object positions on down cam ###

#Hardcode the positions of Lane Marker and Octagon Table based on their location in the Unity sim
octagon_table_pos=[0,0,0]
lane_marker_pos=[0,0,0]


def moveAroundTarget(self): #Make a square around the target
        control.moveDeltaLocal([4,0,0])
        control.rotateEuler([0,0,90])
        control.moveDeltaLocal([4,0,0])
        control.rotateEuler([0,0,90])
        for i in range(2):
            control.moveDeltaLocal([8,0,0])
            control.rotateEuler([0,0,90])
        control.moveDeltaLocal([4,0,0])
        control.kill()

def goToTarget(self, target):
    x,y,z = target
    control.move([x,y,-2], face_destination=True)
    moveAroundTarget()
        

def object_map_cb(msg):
    for obj in msg.objects:
        print(obj)

if __name__ == '__main__':
    rospy.init_node('object_position_test')
    map = ObjectMapper()
    control = Controller(rospy.Time(0))
    sub = rospy.Subscriber('vision/object_map', VisionObjectArray, object_map_cb)

    #Make AUV do a breath first search to find the Lane Marker and Octagon Table
    #BreadthFirstSearch()

    goToTarget(octagon_table_pos)
    goToTarget(lane_marker_pos)

    #See if the object map contains the Lane Marker and Octagon Table at the correct positions
    octagon_table_map_pos = map.getClass(cls="Octagon Table")
    lane_marker_map_pos = map.getClass(cls="Lane Marker")

    print("Actual Octagon Table Position: ", octagon_table_pos)
    print("Octagon Table Position: ", octagon_table_map_pos)
    print("Actual Lane Marker Position: ", lane_marker_pos)
    print("Lane Marker Position: ", lane_marker_map_pos)
    rospy.spin()
    
    