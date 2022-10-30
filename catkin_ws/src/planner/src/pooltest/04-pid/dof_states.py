#!/usr/bin/env python3

import rospy
import smach

from std_msgs.msg import Float64

class SetpointMotion(smach.State):
    def __init__(self, topic, setpoint, duration):
        super().__init__(outcomes=['done'])
        self.setpoint = Float64(setpoint)
        self.duration = duration
        self.topic = topic
        self.pub = rospy.Publisher(topic, Float64, queue_size=50)
    
    def execute(self, ud):
        self.pub.publish(self.setpoint) 
        rospy.sleep(self.duration)
        return 'done'


class Z(SetpointMotion):
    def __init__(self, setpoint, duration=20.0):
        super().__init__('z_setpoint', setpoint, duration)
