#!/usr/bin/env python3
import rospy
from controller import Controller
from std_msgs.msg import Bool

if __name__ == "__main__":
    rospy.init_node("test",log_level=rospy.DEBUG)
    controls = Controller(lambda: rospy.Time.now())
    controls.move((0,0,0))
    controls.kill()