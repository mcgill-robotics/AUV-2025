#!/usr/bin/env python3

import numpy as np
import rospy

from auv_msgs.msg import ThrusterForces, ThrusterMicroseconds
from geometry_msgs.msg import Wrench

d = 0.224 # m
D_1 = 0.895 # m
D_2 = 0.778 # m

# forces produced by T200 thruster at 14V (N)
MAX_FWD_FORCE = 4.52*9.81 
MAX_BKWD_FORCE = -3.52*9.81

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
    wrench_to_thrust maps a Wrench into a force in kg f
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

    #sample wrenches
    #   rostopic pub -1 /effort geometry_msgs/Wrench
    #   "{ force: {x: 20.0, y: 0.0, z: 0.0}, torque:{x: 0.0, y: 0.0, z: 0.0} }"

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

    forces_to_microseconds_cb(tf)

def negativeForceCurve(force):
    return 1.4701043632380542*(10**3) + force*2.3999978362959104*(10**2) + (force**2)*2.5705773429064880*(10**2) + (force**3)*3.1133962497995367*(10**2) + (force**4)*2.1943237103469241*(10**2) + (force**5)*8.4596303821198617*(10**1) + (force**6)*1.6655229499580056*(10**1) + (force**7)*1.3116834437073399

def positiveForceCurve(force):
    return 1.5299083405100268*(10**3) + force*1.9317247519327023*(10**2) + (force**2)*-1.6227874418158476*(10**2) + (force**3)*1.4980771349508325*(10**2) + (force**4)*-8.0478019175136623*(10**1) + (force**5)*2.3661746039755371*(10**1) + (force**6)*-3.5559291204780612 + (force**7)*2.1398707591286295*(10**-1)

def force_to_microseconds(force):
        # cap our input force at maximum fwd/bkwd speeds
        force = min(max(force, MAX_BKWD_FORCE), MAX_FWD_FORCE)
        #two different curves (negative and positive forces)
        if force > 0.0:
            return int(positiveForceCurve(force/9.81))
        elif force < 0.0:
            return int(negativeForceCurve(force/9.81))
        else: return 1500 #middle value is 1500

# functions or relevant code
def forces_to_microseconds_cb(forces_msg):
    # array message to publish
    micro_arr = [None]*8
    micro_arr[ThrusterMicroseconds.SURGE_PORT] = force_to_microseconds(forces_msg.SURGE_PORT)
    micro_arr[ThrusterMicroseconds.SURGE_STAR] = force_to_microseconds(forces_msg.SURGE_STAR) 
    micro_arr[ThrusterMicroseconds.SWAY_BOW] = force_to_microseconds(forces_msg.SWAY_BOW)
    micro_arr[ThrusterMicroseconds.SWAY_STERN] = force_to_microseconds(forces_msg.SWAY_STERN) 
    micro_arr[ThrusterMicroseconds.HEAVE_BOW_PORT] = force_to_microseconds(forces_msg.HEAVE_BOW_PORT) 
    micro_arr[ThrusterMicroseconds.HEAVE_BOW_STAR] = force_to_microseconds(forces_msg.HEAVE_BOW_STAR) 
    micro_arr[ThrusterMicroseconds.HEAVE_STERN_STAR] = force_to_microseconds(forces_msg.HEAVE_STERN_STAR) 
    micro_arr[ThrusterMicroseconds.HEAVE_STERN_PORT] = force_to_microseconds(forces_msg.HEAVE_STERN_PORT)

    micro_msg = ThrusterMicroseconds(micro_arr)
    pub.publish(micro_msg)


if __name__ == '__main__':
    rospy.init_node('thrust_mapper')
    pub = rospy.Publisher('/propulsion/thruster_microseconds', ThrusterMicroseconds, queue_size=5)
    sub = rospy.Subscriber('/effort', Wrench, wrench_to_thrust)

    rospy.spin()
