#!/usr/bin/env python3
import rospy
from controller import Controller
from std_msgs.msg import Bool

if __name__ == "__main__":
    rospy.init_node("test",log_level=rospy.DEBUG)
    controls = Controller(rospy.Time(0))
    print("killing")
    controls.kill()

    controls.move((0,0,0))