#!/usr/bin/env python3

import rospy
from substates.utility.controller import Controller
from std_msgs.msg import Bool

if __name__ == "__main__":
    rospy.init_node("moveForward",log_level=rospy.DEBUG)
    controls = Controller(lambda: rospy.Time.now())
    #print(controls)
    #print(controls.get_superimposer_goal([1,1,0,0,0,0],[Bool(True),Bool(True),Bool(False),Bool(False),Bool(False),Bool(False)],Bool(False)))
    deltaStep = (0,10,0)
    controls.moveDelta(deltaStep)

    rospy.sleep(6)

    print("end")