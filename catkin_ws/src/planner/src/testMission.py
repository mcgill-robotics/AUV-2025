#!/usr/bin/env python3

import rospy
from substates.utility.controller import Controller
from std_msgs.msg import Bool, Empty

if __name__ == "__main__":
    rospy.init_node("moveForward",log_level=rospy.DEBUG)
    controls = Controller(rospy.Time(0))
    controls.move((0,0,0))

    print("end")
