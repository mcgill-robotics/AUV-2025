#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller

rospy.init_node("quali")
rospy.sleep(25)
controls = Controller(rospy.Time(0))
controls.rotateEuler((0,0,None))
controls.moveDelta((None,None,-2))
controls.forceLocal((16,0))
rospy.sleep(60)
controls.forceLocal((0,0))
controls.moveDelta((None,None,2))
controls.kill()
