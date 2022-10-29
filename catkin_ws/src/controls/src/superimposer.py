#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import Vector3, Wrench, WrenchStamped
from std_msgs.msg import Float64, Header
from tf2_ros import Buffer, TransformListener

class Superimposer:
    def __init__(self):
        # force
        self.surge = Superimposer.Degree_Of_Freedom('surge')
        self.sway = Superimposer.Degree_Of_Freedom('sway')
        self.heave = Superimposer.Degree_Of_Freedom('heave')

        # torque
        self.roll = Superimposer.Degree_Of_Freedom('roll')
        self.pitch = Superimposer.Degree_Of_Freedom('pitch')
        self.yaw = Superimposer.Degree_Of_Freedom('yaw')

        # tf2 buffer

        self.pub = rospy.Publisher('effort', Wrench, queue_size=50)

    def update_effort(self, _):
        self.header.stamp = rospy.Time(0)

        # force and torque are vectors in the world ref. frame
        force_world = Vector3(self.surge.val, self.sway.val, self.heave.val)
        torque_world = Vector3(self.roll.val, self.pitch.val, self.yaw.val)
        wrench_world = Wrench(force=force, torque=torque)

	self.pub.publish(wrench)

        # transform is computed on stamped message
        wrench_world_stmp = WrenchStamped(header=self.header, wrench=wrench_world)

        try:
            # force/torque vectors in robot's ref. frame
            wrench_auv = self.tf_buffer.transform(wrench_world_stmp, "auv_base")
            self.pub.publish(wrench_auv.wrench)
        except Exception as e:
            print(type(e), e)


    class Degree_Of_Freedom:
        def __init__(self, sub_topic):
            self.val = 0.0
            rospy.Subscriber(sub_topic, Float64, self.set_cb)

        def set_cb(self, new_val):
            self.val = new_val.data


if __name__ == '__main__':
    rospy.init_node('superimposer')
    si = Superimposer()
    timer = rospy.Timer(rospy.Duration(0.1), si.update_effort)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
