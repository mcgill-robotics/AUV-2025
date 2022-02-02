#!/usr/bin/env python3

import numpy as np
import rospy

from auv_msgs.msg import ThrusterCommand
from geometry_msgs.msg import Wrench

Dx = 3.0
Dy = 1.0
Dz = 2.0

T = np.matrix(
        [[1., 1., 0., 0., 0., 0., 0., 0.],
        [0., 0., 1., 1., 0., 0., 0., 0.],
        [1., 1., 0., 0., 1., 1., 1., 1.],
        [0., 0., 0., 0., -Dy/2, Dy/2, Dy/2, -Dy/2],
        [1., 1., 0., 0., -Dx/2, -Dx/2, Dx/2, Dx/2],
        [Dy/2, -Dy/2, -Dx/2, Dx/2, 0., 0., 0., 0.]]
        )

T_inv = np.linalg.pinv(T) # matrix transformation wrench -> thrust  

def effort_cb():
    thrust_pub = rospy.Publisher('propulsion/thruster_cmd', ThrusterCommand, queue_size=5)
    
    def wrench_to_thrust(w):
        '''
        wrench_to_thrust returns a function, configured with thrust
        characterisations, that maps a Wrench into thrust intensity (TODO)
        [-1.0, 1.0]
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
        tc = ThrusterCommand([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]]) 
        thrust_pub.publish(tc)

    return wrench_to_thrust

if __name__ == '__main__':
    rospy.init_node('thrust_mapper')
    sub = rospy.Subscriber('/effort', Wrench, effort_cb())
    rospy.spin()
