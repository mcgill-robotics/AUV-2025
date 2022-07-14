#!/usr/bin/env python3

import rospy
import smach

from auv_msgs.msg import ThrusterCommand 

class ThrusterMotion(smach.State):
    def __init__(self, cmd, duration=2.0):
        super().__init__(outcomes=['done'])
        self.effort = cmd
        self.duration = duration
        self.pub = rospy.Publisher('propulsion/thruster_cmd', ThrusterCommand, queue_size=50)
    
    def execute(self, ud):
        timer = rospy.Timer(rospy.Duration(0.1), self.update)
        rospy.sleep(self.duration)

        # this is a hack to stop publishing
        timer.shutdown()
        return 'done'

    def update(self, _):
        self.pub.publish(self.effort) 


class Surge(EffortMotion):
    def __init__(self, effort=0.2, duration=10.0):
        cmd = ThrusterCommand([effort, effort, 0, 0, 0, 0, 0, 0])
        super().__init__(cmd, duration)


class Sway(EffortMotion):
    def __init__(self, effort=0.2, duration=10.0):
        cmd = ThrusterCommand([0, 0, effort, effort, 0, 0, 0, 0])
        super().__init__(cmd, duration)


class Heave(EffortMotion):
    def __init__(self, effort=0.2, duration=10.0):
        cmd = ThrusterCommand([0, 0, 0, 0, effort, effort, effort, effort])
        super().__init__(cmd, duration)


class Roll(EffortMotion):
    def __init__(self, effort=0.2, duration=10.0):
        cmd = ThrusterCommand([0, 0, 0, 0, -effort, effort, effort, -effort])
        super().__init__(cmd, duration)


class Pitch(EffortMotion):
    def __init__(self, effort=0.2, duration=10.0):
        cmd = ThrusterCommand([0, 0, 0, 0, effort, effort, -effort, -effort])
        super().__init__(cmd, duration)


class Yaw(EffortMotion):
    def __init__(self, effort=0.2, duration=10.0):
        cmd = ThrusterCommand([-effort, effort, effort, -effort, 0, 0, 0, 0])
        super().__init__(cmd, duration)


class Pause(EffortMotion):

    def __init__(self, effort=0.2, duration=10.0):
        cmd = ThrusterCommand([0, 0, 0, 0, 0, 0, 0, 0])
        super().__init__(cmd, duration)
