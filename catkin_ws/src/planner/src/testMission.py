#!/usr/bin/env python3

import rospy
from substates.utility.controller import Controller
from std_msgs.msg import Bool

if __name__ == "__main__":
    rospy.init_node("moveForward",log_level=rospy.DEBUG)
    controls = Controller(rospy.Time(0))
    controls.move((0,0,0))
    rospy.sleep(2)
    controls.rotate(( 0.707,0.707,0,0))
    controls.move((5,0,0))
    controls.rotate(( 0.707,0,0.707,0))
    controls.move((5,5,0))
    controls.rotate(( 0.707,0,0,0.707))

    print("end")