#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_geometry_msgs

from geometry_msgs.msg import Vector3, Vector3Stamped, Wrench 
from std_msgs.msg import Float64, Header
from tf2_ros import Buffer, TransformListener

class Superimposer:
    def __init__(self):
        # forces in robot reference frame
        self.surge = Superimposer.Degree_Of_Freedom('surge')
        self.sway = Superimposer.Degree_Of_Freedom('sway')
        self.heave = Superimposer.Degree_Of_Freedom('heave')
        self.roll = Superimposer.Degree_Of_Freedom('roll')
        self.pitch = Superimposer.Degree_Of_Freedom('pitch')
        self.yaw = Superimposer.Degree_Of_Freedom('yaw')

        # forces in global reference frame
        self.global_x = Superimposer.Degree_Of_Freedom('global_x')
        self.global_y = Superimposer.Degree_Of_Freedom('global_y')
        self.global_z = Superimposer.Degree_Of_Freedom('global_z')

        # tf2 buffer
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer)

        # avoid creating a new Header object for every update
        # just update the time
        self.header = Header(frame_id="auv_base")

        self.pub_effort = rospy.Publisher('effort', Wrench, queue_size=50)


    def update_effort(self, _):
        '''
        superimposer keeps track of force inputs expressed in both 
        the global, and AUV reference frame. This provides the ability
        for nodes publishing to superimposer to express inputs in either
        reference frame (ie. to go down, the node would publish to 'global_z'
        without having to take into account the orientation of the AUV)
        The 'global' force inputs are translated into the AUV reference frame
        and superimposed with the 'AUV frame' inputs prior to updating 
        effort
        '''

        force_global = Vector3(
                self.global_x.val, self.global_y.val, self.global_z.val)
        force_auv = Vector3(self.surge.val, self.sway.val, self.heave.val)
        torque_auv = Vector3(self.roll.val, self.pitch.val, self.yaw.val)

        self.header.stamp = rospy.Time(0)
        force_global_stmp = Vector3Stamped(
                header=self.header, vector=force_global)

        try:
            # convert global force vector into robot reference frame
            # TODO use message filters to assure tf/data is available
            trans = self.tf_buffer.lookup_transform(
                    "world", "auv_base", rospy.Time(0))

            force_global_transformed = tf2_geometry_msgs.do_transform_vector3(
                    force_global_stmp, trans)

            # adding 'global' and 'AUV' force vectors
            force_auv = Vector3(
                    force_auv.x + force_global_transformed.vector.x, 
                    force_auv.y + force_global_transformed.vector.y, 
                    force_auv.z + force_global_transformed.vector.z)
            
        except Exception as e:
            # TODO: only for specific exceptions where no transforms buffered
            # assume global and AUV reference frames are colinear 
            # (AUV is not rotated), add the vectors without transform
            force_auv = Vector3(
                    force_auv.x + self.global_x.val, 
                    force_auv.y + self.global_y.val, 
                    force_auv.z + self.global_z.val)

            print("exception ---", type(e), e)

        # publish superimposed effort
        effort = Wrench(force=force_auv, torque=torque_auv) 
        self.pub_effort.publish(effort)


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
