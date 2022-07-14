#!/usr/bin/env python3

import rospy
import smach

from geometry_msgs import Twist, Force, Torque

class EffortMotion(smach.State):
    def __init__(self, twist, duration=2.0):
        super().__init__(outcomes=['done'])
        self.effort = twist
        self.duration = duration
        self.pub = rospy.Publisher('effort', Twist, queue_size=50)
    
    def execute(self, ud):
        timer = rospy.Timer(rospy.Duration(0.1), self.update)
        rospy.sleep(self.duration)

        # this is a hack to stop publishing
        timer.shutdown()
        return 'done'

    def update(self, _):
        self.pub.publish(self.effort) 


class Surge(EffortMotion):
    def __init__(self, effort=10.0, duration=10.0):
        force = Force(x=effort, y=0.0, z=0.0)
        torque = Torque(x=0.0, y=0.0, z=0.0)
        twist = Twist(force=force, torque=torque)
        super().__init__(twist, duration)


class Sway(EffortMotion):
    def __init__(self, effort=10.0, duration=10.0):
        force = Force(x=0.0, y=effort, z=0.0)
        torque = Torque(x=0.0, y=0.0, z=0.0)
        twist = Twist(force=force, torque=torque)
        super().__init__(twist, duration)


class Heave(EffortMotion):
    def __init__(self, effort=10.0, duration=10.0):
        force = Force(x=0.0, y=0.0, z=effort)
        torque = Torque(x=0.0, y=0.0, z=0.0)
        twist = Twist(force=force, torque=torque)
        super().__init__(twist, duration)


class Roll(EffortMotion):
    def __init__(self, effort=10.0, duration=10.0):
        force = Force(x=0.0, y=0.0, z=0.0)
        torque = Torque(x=effort, y=0.0, z=0.0)
        twist = Twist(force=force, torque=torque)
        super().__init__(twist, duration)


class Pitch(EffortMotion):
    def __init__(self, effort=10.0, duration=10.0):
        force = Force(x=0.0, y=0.0, z=0.0)
        torque = Torque(x=0.0, y=effort, z=0.0)
        twist = Twist(force=force, torque=torque)
        super().__init__(twist, duration)


class Yaw(EffortMotion):
    def __init__(self, effort=10.0, duration=10.0):
        force = Force(x=0.0, y=0.0, z=0.0)
        torque = Torque(x=0.0, y=0.0, z=effort)
        twist = Twist(force=force, torque=torque)
        super().__init__(twist, duration)


class Pause(EffortMotion):

    def __init__(self, duration=2.0):
        force = Force(x=0.0, y=0.0, z=0.0)
        torque = Torque(x=0.0, y=0.0, z=0.0)
        twist = Twist(force=force, torque=torque)
        super().__init__(twist, duration)
