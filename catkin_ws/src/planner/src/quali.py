#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller

rospy.init_node("quali")
controls = Controller(rospy.Time(0))
controls.rotateEuler((0,0,None))
controls.move((0,0,-2))
controls.moveDelta((8,None,1))
controls.moveDelta((None,8,-2))
controls.rotateDeltaEuler((90,0,0))
controls.rotateDeltaEuler((0,90,0))
controls.rotateDeltaEuler((0,0,-90))
controls.rotateEuler((0,0,0))
controls.rotateEuler((45,0,0))
controls.rotateEuler((0,45,0))
controls.rotateEuler((45,45,270))
controls.moveDelta((None,None,2))
controls.kill()
