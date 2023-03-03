#!/usr/bin/env python3

"""

Description: Thrust mapper node subscribes to effort topic, converts the wrench readings to forces, 
and then finally converts the forces to pwm signals and publishes them.

"""

import numpy as np
import rospy
from thrust_mapper_utils import *
from auv_msgs.msg import ThrusterForces, ThrusterMicroseconds
from geometry_msgs.msg import Wrench


"""---------------------Constants------------------"""
# distance between surge thrusters
d = 0.224
# distance between sway thrusters 
D_1 = 0.895 
# distance (length-wise) between bow and stern
D_2 = 0.778

#Maximum force thrusters can exert in either direction (ie, spin forward or backward)
#Limit to 15% while testing
MAX_FWD_FORCE = 4.52*9.81*0.15 
MAX_BKWD_FORCE = -3.52*9.81*0.15

#Matrix representation of the system of equations representing the thrust to wrench conversion
#Ex: Force_X = (1)Surge_Port_Thruster + (1)Surge_Starboard_Thrust
T = np.matrix(
        [[1., 1., 0., 0., 0., 0., 0., 0.],
        [0., 0., 1., -1., 0., 0., 0., 0.],
        [0., 0., 0., 0., -1., -1., -1., -1.],
        [0., 0., 0., 0., -d/2, d/2, d/2, -d/2],
        [0., 0., 0., 0., -D_2/2, -D_2/2, D_2/2, D_2/2],
        [-d/2, d/2, D_1/2, D_1/2, 0., 0., 0., 0.]]
        )

#matrix transformation wrench -> thrust 
T_inv = np.linalg.pinv(T) 
"""--------------------------------------------------"""
rospy.sleep(7.0) #TODO: FIX - wait for 7 sec to sync with arduino?



def wrench_to_thrust(w):
    '''
    A callback function that maps a Wrench into a force produced by T200 thruster at 14V (N)
    '''
    a = np.array(
            [[w.force.x],
            [w.force.y],
            [w.force.z],
            [w.torque.x],
            [w.torque.y],
            [w.torque.z]]
            )

    converted_w = np.matmul(T_inv, a) 
    tf = ThrusterForces() 

    tf.SURGE_PORT = converted_w[0]
    tf.SURGE_STAR = converted_w[1]
    tf.SWAY_BOW = converted_w[2]
    tf.SWAY_STERN = converted_w[3]
    tf.HEAVE_BOW_PORT = converted_w[4]
    tf.HEAVE_BOW_STAR = converted_w[5]
    tf.HEAVE_STERN_STAR = converted_w[6]
    tf.HEAVE_STERN_PORT = converted_w[7]

    #Convert forces to pwm signals and publish
    forces_to_pwm_publisher(tf)


def forces_to_pwm_publisher(forces_msg):
    """
    Publish pwm signals
    """
    pwm_arr = [None]*8
    pwm_arr[ThrusterMicroseconds.SURGE_PORT] = force_to_pwm(forces_msg.SURGE_PORT)
    pwm_arr[ThrusterMicroseconds.SURGE_STAR] = force_to_pwm(forces_msg.SURGE_STAR) 
    pwm_arr[ThrusterMicroseconds.SWAY_BOW] = force_to_pwm(forces_msg.SWAY_BOW)
    pwm_arr[ThrusterMicroseconds.SWAY_STERN] = force_to_pwm(forces_msg.SWAY_STERN) 
    pwm_arr[ThrusterMicroseconds.HEAVE_BOW_PORT] = force_to_pwm(forces_msg.HEAVE_BOW_PORT) 
    pwm_arr[ThrusterMicroseconds.HEAVE_BOW_STAR] = force_to_pwm(forces_msg.HEAVE_BOW_STAR) 
    pwm_arr[ThrusterMicroseconds.HEAVE_STERN_STAR] = force_to_pwm(forces_msg.HEAVE_STERN_STAR) 
    pwm_arr[ThrusterMicroseconds.HEAVE_STERN_PORT] = force_to_pwm(forces_msg.HEAVE_STERN_PORT)

    pwm_msg = ThrusterMicroseconds(pwm_arr)
    pub.publish(pwm_msg)


if __name__ == '__main__':
    rospy.init_node('thrust_mapper')
    pub = rospy.Publisher('/propulsion/thruster_microseconds', ThrusterMicroseconds, queue_size=5)
    sub = rospy.Subscriber('/effort', Wrench, wrench_to_thrust)

    rospy.spin()
