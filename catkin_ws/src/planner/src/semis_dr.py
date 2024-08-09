#!/usr/bin/env python3
import rospy
from substates.utility.functions import countdown
from substates.utility.controller import *
from substates.utility.state import *
from substates.utility.functions import *


TARGET_FREQUENCY = 40000
PINGER_STEP_DISTANCE = 1
MOVE_FORWARD_FORCE = 20
SLEEPING_TIME = 30

rospy.init_node("semis")
pub_mission_display = rospy.Publisher("/mission_display", String, queue_size=1)


def update_display(status):
    pub_mission_display.publish(status)
    print(status)


countdown(30)

update_display("INIT")
state = StateTracker()
controls = Controller(rospy.Time(0))


update_display("MOVE2GATE")
controls.rotateDeltaEuler([0, 0, 0]) #TODO: change rotation based on flip coin
angle_to_gate = state.theta_z
controls.moveDelta([None, None, -0.75])
controls.moveDelta([1, None, None])


update_display("TRICKS")
for _ in range(2 * 3):
    controls.rotateDeltaEuler((0, 0, 120))


update_display("GATE")
controls.moveDelta([2, None, None])

update_display("NAV TO OCT")
controls.rotateEuler((0, 0, angle_to_gate))
controls.forceLocal((MOVE_FORWARD_FORCE,0,0))

rospy.sleep(SLEEPING_TIME)

controls.kill()
