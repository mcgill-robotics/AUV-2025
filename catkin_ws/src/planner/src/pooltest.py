#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller

rospy.init_node("pooltest")
controls = Controller(rospy.Time(0))

controls.move([0,0,-0.75])
print("FLATTENING")
controls.flatten()
# controls.freeze_pose()
# controls.freeze_position()
# controls.freeze_rotation()

#controls.move([0, 0, -0.75])
# controls.moveDelta([0, 0, -1])
# controls.moveDeltaLocal([0, 0, -1])
# controls.rotate([1, 0, 0, 0])
# controls.rotateDelta([1, 0, 0, 0])
# controls.rotateEuler([0, 0, 180])
# controls.rotateDeltaEuler([0, 0, 180])

while not rospy.is_shutdown():
    continue
controls.kill()
