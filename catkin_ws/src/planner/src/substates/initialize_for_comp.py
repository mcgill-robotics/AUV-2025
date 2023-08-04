#!/usr/bin/env python3

import rospy
import smach
import time
from std_msgs.msg import Empty

class InitializeForComp(smach.State):
    def __init__(self, wait_time=30):
        super().__init__(outcomes=['success'])
        self.wait_time = wait_time

    def execute(self, ud):
        print("Initializing AUV for comp.")
        #MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        # [COMP] MAKE SURE AUV IS IN COORDINATE FRAME WHERE OCTAGON LOCATION WAS MEASURED
        pub_DVL = rospy.Publisher('/reset_state_planar', Empty, queue_size=1)
        rospy.sleep(5)
        pub_DVL.publish(Empty())
        print("Waiting {} seconds.".format(self.wait_time))
        rospy.sleep(self.wait_time)
        print("Initialization complete.")
        return 'success'

