#!/usr/bin/env python3

import rospy

from substates.linear_search import *
from substates.navigate_lane_marker import *
from substates.utility.controller import *
from substates.utility.state import *
from substates.utility.vision import *
from substates.octagon_task import *
from substates.trick import *
from std_msgs.msg import Empty

rospy.init_node("semi_finals_dr")

mapping = ObjectMapper()
state = StateTracker()
control = Controller(rospy.Time(0))

DEPTH = -1
DIST_DOCK_TO_BEFORE_GATE = 2 #for tricks
DIST_DOCK_TO_PIVOT_POINT = 3
YAW_PIVOT_POINT_TO_OCTAGON = 90
DIST_PIVOT_POINT_TO_OCTAGON = 2 # in all

while control.orientation is None:
    pass
    
rospy.sleep(10)

pub_DVL = rospy.Publisher('/reset_state_planar', Empty, queue_size=1)
rospy.sleep(5)
pub_DVL.publish(Empty())
rospy.sleep(5)

yaw = control.theta_z

control.moveDelta((0,0,DEPTH))
control.rotateEuler((0,0,yaw))
control.moveDeltaLocal((DIST_DOCK_TO_BEFORE_GATE,0,0))
trick = Trick(control=control, trick_type="yaw", num_full_spins=2)
trick.execute(None)
control.rotateEuler((0,0,yaw))
control.moveDeltaLocal(((DIST_DOCK_TO_PIVOT_POINT-DIST_DOCK_TO_BEFORE_GATE),0,0))
control.rotateEulerDelta((0,0,YAW_PIVOT_POINT_TO_OCTAGON))
control.moveDeltaLocal((DIST_PIVOT_POINT_TO_OCTAGON,0,0))
control.kill()
