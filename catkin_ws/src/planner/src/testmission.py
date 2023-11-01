#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller

rospy.init_node("test")
controls = Controller(rospy.Time(0))

controls.move((5,0,0))
controls.kill()
