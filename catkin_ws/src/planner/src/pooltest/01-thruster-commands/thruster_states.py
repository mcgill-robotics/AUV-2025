#!/usr/bin/env python3

import rospy
import smach

from auv_msgs.msg import ThrusterIntensities

class ThrusterMotion(smach.State):
    def __init__(self, cmd, duration=2.0):
        super().__init__(outcomes=['done'])
        self.effort = cmd
        self.duration = duration
        self.pub = rospy.Publisher('propulsion/thruster_intensities', ThrusterIntensities, queue_size=50)
    
    def execute(self, ud):
        timer = rospy.Timer(rospy.Duration(0.1), self.update)
        rospy.sleep(self.duration)

        # this is a hack to stop publishing
        timer.shutdown()
        return 'done'

    def update(self, _):
        self.pub.publish(self.effort) 


class Surge(ThrusterMotion):
    def __init__(self, effort=0.2, duration=10.0):
        cmd = ThrusterIntensities([effort, effort, 0, 0, 0, 0, 0, 0])
        super().__init__(cmd, duration)


class Sway(ThrusterMotion):
    def __init__(self, effort=0.2, duration=10.0):
        cmd = ThrusterIntensities([0, 0, effort, effort, 0, 0, 0, 0])
        super().__init__(cmd, duration)


class Heave(ThrusterMotion):
    def __init__(self, effort=0.2, duration=10.0):
        cmd = ThrusterIntensities([0, 0, 0, 0, effort, effort, effort, effort])
        super().__init__(cmd, duration)


class Roll(ThrusterMotion):
    def __init__(self, effort=0.2, duration=10.0):
        cmd = ThrusterIntensities([0, 0, 0, 0, -effort, effort, effort, -effort])
        super().__init__(cmd, duration)


class Pitch(ThrusterMotion):
    def __init__(self, effort=0.2, duration=10.0):
        cmd = ThrusterIntensities([0, 0, 0, 0, effort, effort, -effort, -effort])
        super().__init__(cmd, duration)


class Yaw(ThrusterMotion):
    def __init__(self, effort=0.2, duration=10.0):
        cmd = ThrusterIntensities([-effort, effort, effort, -effort, 0, 0, 0, 0])
        super().__init__(cmd, duration)


class Pause(ThrusterMotion):

    def __init__(self, effort=0.2, duration=10.0):
        cmd = ThrusterIntensities([0, 0, 0, 0, 0, 0, 0, 0])
        super().__init__(cmd, duration)
