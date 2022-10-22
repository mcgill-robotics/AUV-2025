#!/usr/bin/env python3

import numpy as np
import rospy

from auv_msgs.msg import ThrusterCommand
from geometry_msgs.msg import Wrench

d = 0.224 # m
D_1 = 0.895 # m
D_2 = 0.778 # m

thrust_pub = rospy.Publisher('propulsion/thruster_cmd', ThrusterCommand, queue_size=5)
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
    sub = rospy.Subscriber('/effort', Wrench, wrench_to_thrust)
    rospy.spin()
