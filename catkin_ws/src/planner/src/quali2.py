#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller

rospy.init_node("quali")
rospy.sleep(10)
controls = Controller(rospy.Time(0))
pub_DVL = rospy.Publisher('/reset_state_planar', Empty, queue_size=1)
pub_DVL.publish(Empty())
controls.rotateEuler((0,0,None))
controls.moveDelta((None,None,-0.25))
controls.moveDeltaLocal((2,0,None))
controls.moveDelta((None,None,0.25))
controls.kill()
