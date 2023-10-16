#!/usr/bin/env python3

import rospy
from substates.utility.controller import Controller

rospy.init_node("tutorial_dr")

rospy.sleep(25)
controls = Controller(rospy.Time(0))


forward_distance = 5

controls.moveDelta((None,None,-2))
controls.moveDelta((None, forward_distance, None))
controls.rotateEuler((0,0,90))
controls.moveDelta((None, 2, None))
controls.rotateEuler((0,0,90))
controls.moveDeltaLocal((None, -forward_distance, None))
controls.moveDelta((None,None,2))


