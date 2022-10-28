#!/usr/bin/env python3

import numpy as np
import rospy

from auv_msgs.msg import ThrusterForces
from geometry_msgs.msg import Wrench

d = 0.224 # m
D_1 = 0.895 # m
D_2 = 0.778 # m

pub = rospy.Publisher('propulsion/thruster_forces', ThrusterForces, queue_size=5)
rospy.sleep(7.0) #TODO: FIX - wait for 7 sec to sync with arduino?

T = np.matrix(
        [[1., 1., 0., 0., 0., 0., 0., 0.],
        [0., 0., 1., -1., 0., 0., 0., 0.],
        [0., 0., 0., 0., -1., -1., -1., -1.],
        [0., 0., 0., 0., -d/2, d/2, d/2, -d/2],
        [0., 0., 0., 0., -D_2/2, -D_2/2, D_2/2, D_2/2],
        [-d/2, d/2, D_1/2, D_1/2, 0., 0., 0., 0.]]
        )

T_inv = np.linalg.pinv(T) # matrix transformation wrench -> thrust  

def wrench_to_thrust(w):
    '''
    wrench_to_thrust maps a Wrench into an intensity [-1.0, 1.0]
    for each thruster
    '''
    a = np.array(
            [[w.force.x],
            [w.force.y],
            [w.force.z],
            [w.torque.x],
            [w.torque.y],
            [w.torque.z]]
            )

    b = np.matmul(T_inv, a) 
    tf = ThrusterForces() 

    tf.SURGE_PORT = b[0]
    tf.SURGE_STAR = b[1]
    tf.SWAY_BOW = b[2]
    tf.SWAY_STERN = b[3]
    tf.HEAVE_BOW_PORT = b[4]
    tf.HEAVE_BOW_STAR = b[5]
    tf.HEAVE_STERN_STAR = b[6]
    tf.HEAVE_STERN_PORT = b[7]

    pub.publish(tf)


if __name__ == '__main__':
    rospy.init_node('thrust_mapper')
    sub = rospy.Subscriber('/effort', Wrench, wrench_to_thrust)
    rospy.spin()
