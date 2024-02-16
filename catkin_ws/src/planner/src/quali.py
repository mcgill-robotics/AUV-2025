#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller

rospy.init_node("quali")
controls = Controller(rospy.Time(0))

controls.flatten()

# controls.move([0,0,-2])
# controls.rotateDelta([0,1,0,0])
# controls.rotateDeltaEuler([None,0,180])
# controls.rotateDeltaEuler([90,None,None])

# controls.move([0,0,-2])
# controls.rotate([0,1,0,0])
# controls.rotateEuler([0,None,180])
# controls.rotateEuler([None,90,None])

# controls.move([0,0,-2])
# controls.torque([10,10,0])
# rospy.sleep(1)
# controls.rotate([1,0,0,0])

controls.kill()
