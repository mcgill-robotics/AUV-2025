#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import Float64

class TrickEffort(smach.State):
    def __init__(self, control, trick_type, effort=10):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.trick_type = trick_type
        self.effort = effort

    def execute(self,ud):
        #MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        self.control.move((None, None, -2))
        self.control.rotateEuler((0,0,None))
        
        if self.trick_type == "roll":
            return self.execute_roll()
        elif self.trick_type == "pitch":
            return self.execute_pitch()
        elif self.trick_type == "yaw":
            return self.execute_yaw()
    
    def execute_roll(self):
        print("Starting roll trick")
        self.control.torque((self.effort, 0, 0))
        rospy.sleep(2)
        self.control.rotateEulerDelta((0,0,0)) # HACK, this will set the goal to the previous quaternion goal without having to manually save it
        return 'success'
    
    def execute_pitch(self):
        print("Starting pitch trick")
        self.control.torque((0, self.effort, 0))
        rospy.sleep(2)
        self.control.rotateEulerDelta((0,0,0)) # HACK, this will set the goal to the previous quaternion goal without having to manually save it
        print("Completed")
        return 'success'
    
    def execute_yaw(self):
        print("Starting yaw trick")
        self.control.torque((0, 0, self.effort))
        rospy.sleep(2)
        self.control.rotateEulerDelta((0,0,0)) # HACK, this will set the goal to the previous quaternion goal without having to manually save it
        print("Completed")
        return 'success'