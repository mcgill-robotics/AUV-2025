#!/usr/bin/env python3

import rospy

import numpy as np
from std_msgs.msg import Float64MultiArray


def cb_hydrophones(msg):
    dt_h1 = msg.data[0]
    dt_h2 = msg.data[1]

    measurements = calculate_time_measurements(dt_h1, dt_h2) 
    bearing_vector = solve_bearing_vector(measurements[0], measurements[1])
    bearing_vector_msg = Float64MultiArray()
    bearing_vector_msg.data = bearing_vector
    pub_pinger_direction.publish(bearing_vector_msg)

#With 4 hydrophones
def solve_bearing_vector(H_matrix, measurements, num):
    H_matrix = np.array(H_matrix)
    measurements = np.array(measurements)

    # Check if invertible
    if np.linalg.matrix_rank(H_matrix) < num:
        raise ValueError("Not invertible H_matrix")

    bearing_vector = np.linalg.inv(H_matrix) @ measurements

    return bearing_vector

def calculate_time_measurements(t0,t1,t2,t3):
    '''
    tn = time of arrival at hydrophone n
    c = speed of sound in medium
    dn = c(t0-tn)
    Take t0 as the reference time (time when sound wave hits hydrophone 0)
    '''
    d1 = c*(t0-t1)
    d2 = c*(t0-t2)
    d3 = c*(t0-t3)
    return np.array([d1,d2,d3])

#With 3 hydrophones, placed on x-y axes
def calculate_time_measurements(dt1,dt2):
    '''
    c = speed of sound in medium
    dn = distance from hydrophone 1 to hydrophone n
    '''
    d1 = c * dt1
    d2 = c * dt2
    return np.array([d1,d2])


def solve_bearing_vector(dx,dy):
    bearing_vector = np.array([dx/x, dy/y])
    return bearing_vector


if __name__ == "__main__":
    rospy.init_node("hydrophones_bearing")
    rospy.Subscriber("/sensors/hydrophones", Float64MultiArray, cb_hydrophones)
    pub_pinger_direction = rospy.Publisher("/sensors/hydrophones/pinger1_bearing", Float64MultiArray, queue_size=1)

    # Speed of sound in water
    c = 1480

    # H_matrix = np.array([
    #     [H1x, H1y, H1z],
    #     [H2x, H2y, H2z],
    #     [H3x, H3y, H3z]
    # ])
    # H_matrix = np.array([
    #     [0.00,  0.00, -0.5],
    #     [0.05,  0.00, -0.5],
    #     [0.00,  0.05, -0.5]
    # ])

    # Assume H1 is the "origin" hydrophone
    # If you change the values here, you must also change the values in the Unity editor
    # H1 is at the origin, H2 is on the x-axis, H3 is on the y-axis
    x = 0.05
    y = 0.05

    rospy.spin()

    

    