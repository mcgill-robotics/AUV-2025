#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller

rospy.init_node("quali")
controls = Controller(rospy.Time(0))
controls.rotateEuler((0,0,None))
controls.move((0,0,-2))
controls.rotateDeltaEuler((90,0,0))
controls.rotateDeltaEuler((0,90,0))
controls.rotateDeltaEuler((-90,0,0))
controls.moveDelta((None,None,2))
controls.kill()
