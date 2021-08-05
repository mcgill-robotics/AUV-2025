#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Wrench, Vector3
from std_msgs.msg import Float64


thrust_decay = rospy.get_param('controls/thrust_decay', 1.0)


class SuperImposer:
    def __init__(self):
        self.thrust_pub = rospy.Publisher('controls/wrench',
                                          Wrench,
                                          queue_size=10)

        self.force = Vector3(0, 0, 0)
        self.torque = Vector3(0, 0, 0)

        # translation subscribers
        rospy.Subscriber('controls/superimposer/surge',
                         Float64,
                         lambda data: self.set_surge(data.data))
        rospy.Subscriber('controls/superimposer/sway',
                         Float64,
                         lambda data: self.set_sway(data.data))
        rospy.Subscriber('controls/superimposer/heave',
                         Float64,
                         lambda data: self.set_heave(data.data))

        # rotation subscribers
        rospy.Subscriber('controls/superimposer/roll',
                         Float64,
                         lambda data: self.set_roll(data.data))
        rospy.Subscriber('controls/superimposer/pitch',
                         Float64,
                         lambda data: self.set_pitch(data.data))
        rospy.Subscriber('controls/superimposer/yaw',
                         Float64,
                         lambda data: self.set_yaw(data.data))

    def update(self, _):
        thrust_wrench = Wrench(self.force, self.torque)
        self.thrust_pub.publish(thrust_wrench)

        # Time decaying forces
        self.force.x = self.force.x * thrust_decay
        self.force.y = self.force.y * thrust_decay
        self.force.z = self.force.z * thrust_decay

        self.torque.x = self.torque.x * thrust_decay
        self.torque.y = self.torque.y * thrust_decay
        self.torque.z = self.torque.z * thrust_decay

    def reset(self):
        self.force = Vector3(0, 0, 0)
        self.torque = Vector3(0, 0, 0)

    # setters for translational motion
    def set_surge(self, surge):
        self.force.x = surge

    def set_sway(self, sway):
        self.force.y = sway

    def set_heave(self, heave):
        self.force.z = heave

        # Positive buonancy eliminates the need for this
        if self.force.z < 0:
            self.force.z = 0.0

    # setters for rotational motion
    def set_roll(self, roll):
        self.torque.x = roll

    def set_pitch(self, pitch):
        self.torque.y = pitch

    def set_yaw(self, yaw):
        self.torque.z = yaw


if __name__ == '__main__':
    rospy.init_node('controls')

    super_imposer = SuperImposer()

    # TODO: experiment with referesh rate
    timer = rospy.Timer(rospy.Duration(0.1), super_imposer.update)
    rospy.on_shutdown(timer.shutdown)

    rospy.spin()
