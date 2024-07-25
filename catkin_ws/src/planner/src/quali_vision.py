#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller
from substates.utility.state import StateTracker
from substates.utility.vision import ObjectMapper
from substates.navigate_gate import *

rospy.init_node("quali_vision")

controls = Controller(rospy.Time(0))
state = StateTracker()
mapping = ObjectMapper()

quali_gate_width = rospy.get_param("quali_gate_width")
target_symbol = rospy.get_param("target_symbol")
nominal_depth = rospy.get_param("nominal_depth")

print("Submerging")
controls.move((None, None, nominal_depth))

print("Navigating gate")
gateNav = NavigateGate(
    controls,
    state,
    mapping,
    target_symbol,
    goThrough=False,
    gate_width=quali_gate_width,
)
gateNav.execute(None)

print("Moving to right side of gate")
controls.moveDeltaLocal((0, -quali_gate_width / 4, 0))

print("Moving through gate")
controls.moveDeltaLocal((14, 0, 0))

print("Rotating around pole")
controls.rotateDeltaEuler((0, 0, 90))

controls.moveDeltaLocal((quali_gate_width / 2, 0, 0))

controls.rotateDeltaEuler((0, 0, 90))

print("Returning to origin")
controls.moveDeltaLocal((17, 0, 0))

controls.kill()
