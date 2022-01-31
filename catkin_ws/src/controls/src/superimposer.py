#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Vector3, Wrench
from std_msgs.msg import Float64

class Superimposer:
    def __init__(self):
        # force
        self.heave = self.Degree_Of_Freedom('heave')
        self.surge = self.Degree_Of_Freedom('surge')
        self.sway = self.Degree_Of_Freedom('sway')

        # torque
        self.pitch = self.Degree_Of_Freedom('pitch')
        self.roll = self.Degree_Of_Freedom('roll')
        self.yaw = self.Degree_Of_Freedom('yaw')

        self.pub = rospy.Publisher('effort', Wrench, queue_size=50)

    def update_effort(self, _):
        # TODO - make better use of second param
        force = Vector3(self.surge.get(), self.sway.get(), self.heave.get())
        torque = Vector3(self.roll.get(), self.pitch.get(), self.yaw.get())
        wrench = Wrench(force, torque)
        self.pub.publish(wrench)

    class Degree_Of_Freedom:
        def __init__(self, sub_topic):
            self.val = 0.0
            rospy.Subscriber(sub_topic, Float64, self.set_cb)
        
        def get(self):
            return self.val

        def set_cb(self, new_val):
            self.val = new_val.data

if __name__ == '__main__':
    rospy.init_node('superimposer')
    si = Superimposer()
    timer = rospy.Timer(rospy.Duration(0.1), si.update_effort)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
