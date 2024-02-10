#!/usr/bin/env python3

import rospy
import smach
from auv_msgs.msg import VisionObjectArray

import time

from substates.breadth_first_search import *
from substates.utility.controller import *
from substates.utility.state import *
from substates.utility.vision import *

def endPlanner(msg="Shutting down mission planner."):
    print(msg)
    control.kill()

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

def goToTarget(target):
    print("Moving to target")
    x,y,z = target
    control.move([x,y,-2], face_destination=True)
    print("Arrived at target")
    #moveAroundTarget()

def object_map_cb(msg):
    for obj in msg.objects:
        print(obj)

if __name__ == '__main__':
    rospy.init_node('object_position_test',log_level=rospy.DEBUG)
    sub = rospy.Subscriber('vision/object_map', VisionObjectArray, object_map_cb)
    rospy.on_shutdown(endPlanner)

    try:
        mapping = ObjectMapper()
        state = StateTracker()
        control = Controller(rospy.Time(0))
        sm = None

        print("STARTING")

        #Go to approximate position of the Octagon Table and Lane Marker
        octagon_table_pos=[-4.706,-9.036,-2.486]
        lane_marker_pos=[1.661,6.204,-2.363]
        lane_marker_pos2=[9.882, 1.281, 0]

        goToTarget(octagon_table_pos)
        goToTarget(lane_marker_pos)
        goToTarget(lane_marker_pos2)

        time.sleep(5)

        control.kill()

        octagon_table_map_pos = mapping.getClass(cls="Octagon Table")
        lane_marker_map_pos = mapping.getClass(cls="Lane Marker")

        print("Octagon Table Position: ", octagon_table_map_pos)
        print("Lane Marker Position(s): ", lane_marker_map_pos)

    except KeyboardInterrupt:
        if sm is not None: sm.request_preempt()
    finally:
        endPlanner()