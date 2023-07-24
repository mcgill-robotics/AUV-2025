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
import math

# distances in m
dx_1 = 0.397 # T1 -> T2
dx_2 = 0.395 # T7 -> T8, T5 -> T6 
dy_1 = 0.514 # T6 -> T7, T5 -> T8
dy_2 = 0.779 # perpendicular T3 -> T4


T = np.matrix(
        [[    -1.,     -1.,   0.,   0.,      0.,     0.,     0.,       0.],
        [      0.,      0.,   1.,  -1.,      0.,     0.,     0.,       0.],
        [      0.,      0.,   0.,   0.,     -1.,    -1.,    -1.,      -1.],
        [      0.,      0.,   0.,   0., -dx_2/2, dx_2/2,  dx_2/2, -dx_2/2],
        [      0.,      0.,   0.,   0.,  dy_1/2, dy_1/2, -dy_1/2, -dy_1/2],
        [  dx_1/2, -dx_1/2, dy_2, dy_2,      0.,     0.,      0.,      0.]]
        )

# forces produced by T200 thruster at 14V (N)
THRUST_LIMIT = 1.0  # Limit thruster speed while dry-testing
MAX_FWD_FORCE = 4.52*9.81*THRUST_LIMIT
MAX_BKWD_FORCE = -3.52*9.81*THRUST_LIMIT

#Matrix representation of the system of equations representing the thrust to wrench conversion
#Ex: Force_X = (1)Surge_Port_Thruster + (1)Surge_Starboard_Thrust

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

    # this is used by the sim
    pub_forces.publish(tf)

    #Convert forces to pwm signals and publish
    forces_to_pwm_publisher(tf)


def forces_to_pwm_publisher(forces_msg):
    """
    Publish pwm signals
    """
    pwm_arr = [None]*8
    pwm_arr[ThrusterMicroseconds.SURGE_PORT] = force_to_pwm(forces_msg.SURGE_PORT,MAX_FWD_FORCE,MAX_BKWD_FORCE)
    pwm_arr[ThrusterMicroseconds.SURGE_STAR] = force_to_pwm(forces_msg.SURGE_STAR,MAX_FWD_FORCE,MAX_BKWD_FORCE) 
    pwm_arr[ThrusterMicroseconds.SWAY_BOW] = force_to_pwm(forces_msg.SWAY_BOW,MAX_FWD_FORCE,MAX_BKWD_FORCE)
    pwm_arr[ThrusterMicroseconds.SWAY_STERN] = force_to_pwm(forces_msg.SWAY_STERN,MAX_FWD_FORCE,MAX_BKWD_FORCE) 
    pwm_arr[ThrusterMicroseconds.HEAVE_BOW_PORT] = force_to_pwm(forces_msg.HEAVE_BOW_PORT,MAX_FWD_FORCE,MAX_BKWD_FORCE) 
    pwm_arr[ThrusterMicroseconds.HEAVE_BOW_STAR] = force_to_pwm(forces_msg.HEAVE_BOW_STAR,MAX_FWD_FORCE,MAX_BKWD_FORCE) 
    pwm_arr[ThrusterMicroseconds.HEAVE_STERN_STAR] = force_to_pwm(forces_msg.HEAVE_STERN_STAR,MAX_FWD_FORCE,MAX_BKWD_FORCE) 
    pwm_arr[ThrusterMicroseconds.HEAVE_STERN_PORT] = force_to_pwm(forces_msg.HEAVE_STERN_PORT,MAX_FWD_FORCE,MAX_BKWD_FORCE)
    
    # TODO - these are temporary precautionary measures and may result in unwanted dynamics
    # so as not to trip individual fuse, thruster current draw is limited to < 9.5 A
    # 1228 <= PWM <= 1768 
    for i in range(len(pwm_arr)):
        if pwm_arr[i] > 1768:
            pwm_arr[i] = 1768
            print("INDIVIDUAL FUSE EXCEEDED: T", i+1)
        elif pwm_arr[i] < 1228:
            pwm_arr[i] = 1228
            print("INDIVIDUAL FUSE EXCEEDED: T", i+1)

    pwm_msg = ThrusterMicroseconds(pwm_arr)
    pub_us.publish(pwm_msg)

#turns off the thursters when the node dies
def shutdown():
    msg = ThrusterMicroseconds([1500]*8)
    pub_us.publish(msg)

#sends the arming signal to the thursters upon startup
def re_arm():
    rospy.sleep(1)
    msg1  = ThrusterMicroseconds([1500]*8)
    msg2 = ThrusterMicroseconds([1540]*8)

    pub_us.publish(msg1)
    rospy.sleep(0.5)
    pub_us.publish(msg2)
    rospy.sleep(0.5)
    pub_us.publish(msg1)

if __name__ == '__main__':
    rospy.init_node('thrust_mapper')
    pub_us = rospy.Publisher('/propulsion/thruster_microseconds', ThrusterMicroseconds, queue_size=1)
    pub_forces = rospy.Publisher('/propulsion/thruster_forces', ThrusterForces, queue_size=1)
    rospy.Subscriber('/effort', Wrench, wrench_to_thrust)
    rospy.on_shutdown(shutdown)
    re_arm()
    rospy.spin()
