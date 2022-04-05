#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_geometry_msgs # needed for do_transform_wrench

from geometry_msgs.msg import Vector3, Wrench, WrenchStamped 
from std_msgs.msg import Float64, Header
from tf2_ros import Buffer, TransformListener 

class Superimposer:
    def __init__(self):
        # force
        self.heave = Superimposer.Degree_Of_Freedom('heave')
        self.surge = Superimposer.Degree_Of_Freedom('surge')
        self.sway = Superimposer.Degree_Of_Freedom('sway')

        # torque
        self.axial_effort = Superimposer.Degree_Of_Freedom('axial_effort')
        self.axis_of_rot = Superimposer.Axis('dtheta_state_axis')  

        # tf2 buffer
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer) 
        self.pub = rospy.Publisher('effort', Wrench, queue_size=50)

        # avoid creating a new Header object for every update
        # just update the time
        self.header = Header(frame_id="world")

        # give time for listener to gather transform data
        # rospy.sleep(15.0)

    def update_effort(self, _):
        self.header.stamp = rospy.Time(0)

        # force and torque are vectors in the world ref. frame
        # they need to be converted into the ref. frame of the robot
        # before being published as wrench
        force_world = Vector3(self.surge.val, self.sway.val, self.heave.val)
        t = np.zeros(3, dtype=np.float64) \
                if self.axis_of_rot.unit_vector is None \
                else self.axial_effort.val * self.axis_of_rot.unit_vector
        torque_world = Vector3(*t)
        wrench_world = Wrench(force=force_world, torque=torque_world) 
  
        # transform is computed on stamped message
        wrench_world_stmp = WrenchStamped(header=self.header, wrench=wrench_world)

        try:
            # TODO use message filters to assure tf/data is available
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
