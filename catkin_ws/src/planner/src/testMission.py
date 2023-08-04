#!/usr/bin/env python3

import rospy
from substates.utility.controller import Controller
from std_msgs.msg import Bool, Empty

if __name__ == "__main__":
    rospy.init_node("moveForward",log_level=rospy.DEBUG)
    controls = Controller(rospy.Time(0))
    pub_DVL = rospy.Publisher('/reset_state_planar', Empty, queue_size=10)
    pub_DVL.publish(Empty())
    controls.rotateDeltaEuler((0,0,0))
    controls.moveDelta((0,0,-0.25))
    controls.rotateDeltaEuler((0,0,120))
    controls.rotateDeltaEuler((0,0,120))
    controls.rotateDeltaEuler((0,0,120))
    controls.rotateDeltaEuler((0,0,120))
    controls.rotateDeltaEuler((0,0,120))
    controls.rotateDeltaEuler((0,0,120))

    #controls.move((0,0,-2))
    #controls.rotate((1,0,0,0))
    #rospy.sleep(60)
    #controls.moveDelta((0,1,0))
    #controls.rotateEuler((0,0,30))
    #rospy.sleep(60)
    #controls.rotateEuler((0,0,-30))
    #rospy.sleep(60)
    #controls.rotate((1,0,0,0))
    #controls.moveDelta((0,-2,0))
    #rospy.sleep(60)
    #controls.rotateEuler((0,0,30))
    #rospy.sleep(60)
    #controls.rotateEuler((0,0,-30))
    controls.kill()

    print("end")
