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

DIST_DOCK_TO_BEFORE_GATE = 5 #for tricks
YAW_GATE_TO_OCTAGON = 15 # in all
DIST_GATE_TO_OCTAGON = 15 # in all

mapping = ObjectMapper()
state = StateTracker()
control = Controller(rospy.Time(0))

#TODO!!!! [COMP] CHANGE DEPTHS to -2 IN NAV LM AND LINEAR SEARCH + DISTANCE OF MOVE DELTAS, 
while control.orientation is None:
    pass
rospy.sleep(10)

pub_DVL = rospy.Publisher('/reset_state_planar', Empty, queue_size=1)
rospy.sleep(5)
pub_DVL.publish(Empty())
rospy.sleep(5)

control.rotateEuler((0,0,None))
control.moveDelta((0,0,-3))
rospy.sleep(10)
control.moveDeltaLocal((5,0,0))
control.rotate(orientation)
trick = Trick(control=control, trick_type="yaw", num_full_spins=2)
trick.execute(None)
control.rotate(orientation)
rospy.sleep(5)
control.moveDeltaLocal((13,0,0))
control.kill()
