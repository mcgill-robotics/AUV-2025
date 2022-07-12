#!/usr/bin/env python3

import rospy
import smach

from std_msgs.msg import Float64

class DeadReckonMotion(smach.State):
    def __init__(self, topic, effort, duration=2.0):
        super().__init__(outcomes=['done'])
        self.effort = Float64(effort)
        self.duration = duration
        self.topic = topic
        self.pub = rospy.Publisher(topic, Float64, queue_size=50)
    
    def execute(self, ud):
        timer = rospy.Timer(rospy.Duration(0.1), self.update)
        rospy.sleep(self.duration)

        # this is a hack to stop publishing
        timer.shutdown()
        return 'done'

    def update(self, _):
        print("publishing to ", self.topic)
        self.pub.publish(self.effort) 

class Pause(smach.State):
    off = Float64(0.0)
    pub_heave = rospy.Publisher('heave', Float64, queue_size=50)    
    pub_sway = rospy.Publisher('sway', Float64, queue_size=50)    
    pub_surge = rospy.Publisher('surge', Float64, queue_size=50)    
    pub_yaw = rospy.Publisher('yaw', Float64, queue_size=50)    
    pub_roll = rospy.Publisher('roll', Float64, queue_size=50)    
    pub_pitch = rospy.Publisher('pitch', Float64, queue_size=50)    

    def __init__(self, duration=0.0):
        super().__init__(outcomes=['done'])
        self.duration = duration

    def execute(self, ud):
        Pause.pub_heave.publish(Pause.off)
        Pause.pub_sway.publish(Pause.off)
        Pause.pub_surge.publish(Pause.off)
        Pause.pub_yaw.publish(Pause.off)
        Pause.pub_roll.publish(Pause.off)
        Pause.pub_pitch.publish(Pause.off)

        rospy.sleep(self.duration)
        return 'done'
