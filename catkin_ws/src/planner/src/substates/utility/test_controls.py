#!/usr/bin/env python3
import rospy
from controller import Controller
from std_msgs.msg import Bool

if __name__ == "__main__":
    rospy.init_node("test",log_level=rospy.DEBUG)
    controls = Controller()
    print(controls)
    print(controls.get_superimposer_goal([1,1,0,0,0,0],[Bool(True),Bool(True),Bool(False),Bool(False),Bool(False),Bool(False)],Bool(False)))
    controls.velocity([1,1,1])