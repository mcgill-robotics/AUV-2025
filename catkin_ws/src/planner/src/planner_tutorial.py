#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller

forward_distance = 15
rospy.init_node("planner_tutorial")
rospy.sleep(25)

controller = Controller(rospy.Time(0))
controller.move([None, None, -2])
controller.move([forward_distance, None, None])
controller.rotateDeltaEuler([0, 0, 90])
controller.move([None, 2, None])
controller.rotateDeltaEuler([0, 0, 90])
controller.move([0, None, None])
controller.kill()
