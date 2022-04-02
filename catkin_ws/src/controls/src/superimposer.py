#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import Vector3, Wrench
from std_msgs.msg import Float64

class Superimposer:
    def __init__(self):
        # force
        self.heave = Superimposer.Degree_Of_Freedom('heave')
        self.surge = Superimposer.Degree_Of_Freedom('surge')
        self.sway = Superimposer.Degree_Of_Freedom('sway')

        # torque
        self.axial_effort = Superimposer.Degree_Of_Freedom('axial_effort')
        self.axis_of_rot = Superimposer.Axis('dtheta_state_axis')  

        self.pub = rospy.Publisher('effort', Wrench, queue_size=50)

    def update_effort(self, _):
        force = Vector3(self.surge.val, self.sway.val, self.heave.val)
        
        if self.axis_of_rot.unit_vector is None:
            t = np.zeros(3, dtype=np.float64) # no rotational force
        else:
            t = self.axial_effort.val * self.axis_of_rot.unit_vector

        torque = Vector3(*t)
        wrench = Wrench(force, torque)
        self.pub.publish(wrench)

    class Degree_Of_Freedom:
        def __init__(self, sub_topic):
            self.val = 0.0
            rospy.Subscriber(sub_topic, Float64, self.set_cb)
        
        def set_cb(self, new_val):
            self.val = new_val.data


    class Axis:
        def __init__(self, sub_topic):
            self.unit_vector = None 
            rospy.Subscriber(sub_topic, Vector3, self.set_cb)
        
        def set_cb(self, new_val):
            # right now being trusting of source that vector is normalized
            self.unit_vector = np.array([new_val.x, new_val.y, new_val.z], dtype=np.float64)


if __name__ == '__main__':
    rospy.init_node('superimposer')
    si = Superimposer()
    timer = rospy.Timer(rospy.Duration(0.1), si.update_effort)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
