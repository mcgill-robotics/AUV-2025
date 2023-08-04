#!/usr/bin/env python3

import rospy

from substates.linear_search import *
from substates.navigate_lane_marker import *
from substates.utility.controller import *
from substates.utility.state import *
from substates.utility.vision import *
from substates.octagon_task import *
from substates.trick import *

rospy.init_node("semi_finals_dr")

mapping = ObjectMapper()
state = StateTracker()
control = Controller(rospy.Time(0))

#TODO!!!! [COMP] CHANGE DEPTHS to -2 IN NAV LM AND LINEAR SEARCH + DISTANCE OF MOVE DELTAS, 
rospy.sleep(30)
control.moveDelta((0,0,-1))
control.rotateEuler((0,0,None))
control.moveDeltaLocal((4,0,0))
trick = Trick(control=control, trick_type="yaw", num_full_spins=2)
trick.execute(None)
linear_search = LinearSearch(timeout=120, forward_speed=10, target_class="Lane Marker", min_objects=1, control=control, mapping=mapping)
linear_search.execute(None)
rospy.sleep(1)
control.moveDeltaLocal((1,0,0))
rospy.sleep(1)
nav_lm = NavigateLaneMarker(origin_class="", control=control, mapping=mapping, state=state)
nav_lm.execute(None)
rospy.sleep(1)
linear_search = LinearSearch(timeout=120, forward_speed=10, target_class="Lane Marker", min_objects=2, control=control, mapping=mapping)
linear_search.execute(None)
rospy.sleep(1)
control.moveDeltaLocal((1,0,0))
rospy.sleep(1)
nav_lm = NavigateLaneMarker(origin_class="Lane Marker", control=control, mapping=mapping, state=state)
nav_lm.execute(None)
rospy.sleep(1)
control.rotateDeltaEuler((0,0,18))
control.moveDeltaLocal((15,0,0))
rospy.sleep(1)
control.kill()