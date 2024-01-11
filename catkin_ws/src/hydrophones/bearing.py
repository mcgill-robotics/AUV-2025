#!/usr/bin/env python3

import numpy as np
from auv_msgs.msg import UnityState


def cb_hydrophones(msg):
    t1 = msg.hydrophones_time_diff[0]
    t2 = msg.hydrophones_time_diff[1]
    t3 = msg.hydrophones_time_diff[2]

#With 4 hydrophones

def solve_bearing_vector(H_matrix, measurements, num):
    H_matrix = np.array(H_matrix)
    measurements = np.array(measurements)

    # Check if invertible
    if np.linalg.matrix_rank(H_matrix) < num:
        raise ValueError("Not invertible H_matrix")

    bearing_vector = np.linalg.inv(H_matrix) @ measurements

    return bearing_vector

def calculate_time_measurements(t0,t1,t2,t3, c):
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


print("Bearing Vector:", bearing_vector)

#With 3 hydrophones, placed on x-y axes
def calculate_time_measurements(t0,t1,t2, c):
    '''
    tn = time of arrival at hydrophone n
    c = speed of sound in medium
    dn = c(t0-tn)
    Take t0 as the reference time
    '''
    d1 = c*(t0-t1)
    d2 = c*(t0-t2)
    return np.array([d1,d2])


def solve_bearing_vector(x,y, dx,dy):
    bearing_vector = np.array([dx/x, dy/y])
    return bearing_vector


if __name__ == "__main__":
    rospy.init_node("hydrophones")

    rospy.Subscriber("/unity/state", UnityState, cb_hydrophones)

    # H_matrix = np.array([
    #     [H1x, H1y, H1z],
    #     [H2x, H2y, H2z],
    #     [H3x, H3y, H3z]
    # ])

    # measurements = np.array([d1, d2, d3])

    # Example usage for the simplified case where the hydrophones are along the x,y,z axes:
    H_matrix = np.array([
        [0.1,0,0],
        [0,0.1,0],
        [0,0,0.1]
    ])

    measurements = calculate_time_measurements(2,1.8,1.4,1.9, 1480) 
    bearing_vector = solve_bearing_vector(H_matrix, measurements, 3)