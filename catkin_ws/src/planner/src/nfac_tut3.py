#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller

rospy.init_node("nfac")
controls = Controller(rospy.Time(0))

#descend 2 meters
controls.moveDelta([0,0,-2])

#move forward 3 meters
controls.moveDeltaLocal([3,0,0])

#rorate 90 degrees to the left
controls.rotateEuler([0,0,90])

#move the auv to its left (forward)
controls.moveDeltaLocal([3,0,0])

#rotate 90 degrees to the left
controls.rotateEuler([0,0,90])

#move the auv backward (forward again)
controls.moveDeltaLocal([3,0,0])

#kill controlls and float to surface
controls.kill()