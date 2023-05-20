#!/usr/bin/env python3

import rospy
import smach

from substates.breadth_first_search import *
from substates.grid_search import *
from substates.linear_search import *
from substates.navigate_lane_marker import *
from substates.test_submerged_rotations import *
from substates.utility.controller import Controller





# LIST OF TASKS:
# 1. "Oh for crying out loud" ==> Coin flip determines the starting orientation (90 or 180 degrees 
#                                 from dock). Go through the gate 
# 2. "Destination"            ==> Choose one of the images (Abydos or Earth) and go throught the gate.
#                                 Extra points for staying with your destination for the remainder of 
#                                 the tasks. 
#                                 With style: 3 meters before and after gate
#                                             Evert 90 degrees change in orientation
#                                               i) Yaw = +100 pts
#                                               ii) Roll/Pitch = +200 pts
#                                             Up to 800 pts
# 3. "Start dialing"
# 4. "Location"
# 5. "Goa'uld attack"
# 6. "Engaging chevrons"
# 7. "DHD"
