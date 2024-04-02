#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller

rospy.init_node("quali")
controls = Controller(rospy.Time(0))

controls.flatten()

# TODO

controls.kill()
