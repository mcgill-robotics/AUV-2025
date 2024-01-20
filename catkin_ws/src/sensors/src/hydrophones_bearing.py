#!/usr/bin/env python3

import rospy

import numpy as np
from auv_msgs.msg import PingerBearing, PingerTimeDifference


def cb_hydrophones_time_difference(msg):
    PingerBearing_msg = PingerBearing()

    if msg.is_pinger1_active:
        dt1_h1 = msg.dt_pinger1[0]
        dt1_h2 = msg.dt_pinger1[1]
        measurements = calculate_time_measurements(dt1_h1, dt1_h2) 
        bearing_vector = solve_bearing_vector(measurements[0], measurements[1])
        PingerBearing_msg.pinger1_bearing.x = bearing_vector[0]
        PingerBearing_msg.pinger1_bearing.y = bearing_vector[1]
        PingerBearing_msg.pinger1_bearing.z = 0

    if msg.is_pinger2_active:
        dt2_h1 = msg.dt_pinger2[0]
        dt2_h2 = msg.dt_pinger2[1]
        measurements = calculate_time_measurements(dt2_h1, dt2_h2)
        bearing_vector = solve_bearing_vector(measurements[0], measurements[1])
        PingerBearing_msg.pinger2_bearing.x = bearing_vector[0]
        PingerBearing_msg.pinger2_bearing.y = bearing_vector[1]
        PingerBearing_msg.pinger2_bearing.z = 0

    if msg.is_pinger3_active:
        dt3_h1 = msg.dt_pinger3[0]
        dt3_h2 = msg.dt_pinger3[1]
        measurements = calculate_time_measurements(dt3_h1, dt3_h2)
        bearing_vector = solve_bearing_vector(measurements[0], measurements[1])
        PingerBearing_msg.pinger3_bearing.x = bearing_vector[0]
        PingerBearing_msg.pinger3_bearing.y = bearing_vector[1]
        PingerBearing_msg.pinger3_bearing.z = 0

    if msg.is_pinger4_active:
        dt4_h1 = msg.dt_pinger4[0]
        dt4_h2 = msg.dt_pinger4[1]
        measurements = calculate_time_measurements(dt4_h1, dt4_h2)
        bearing_vector = solve_bearing_vector(measurements[0], measurements[1])
        PingerBearing_msg.pinger4_bearing.x = bearing_vector[0]
        PingerBearing_msg.pinger4_bearing.y = bearing_vector[1]
        PingerBearing_msg.pinger4_bearing.z = 0
    
    pub_pinger_bearing.publish(PingerBearing_msg)


#With 3 hydrophones, placed on x-y axes
def calculate_time_measurements(dt1, dt2):
    '''
    c = speed of sound in medium
    dn = distance from hydrophone 1 to hydrophone n
    '''
    d1 = c * dt1
    d2 = c * dt2
    return [d1,d2]


def solve_bearing_vector(dx, dy):
    bearing_vector = [dx/x, dy/y]
    return bearing_vector


if __name__ == "__main__":
    rospy.init_node("hydrophones_bearing")
    rospy.Subscriber("/sensors/hydrophones/pinger_time_difference", PingerTimeDifference, cb_hydrophones_time_difference)
    pub_pinger_bearing = rospy.Publisher("/sensors/hydrophones/pinger_bearing", PingerBearing, queue_size=1)

    # Speed of sound in water
    c = 1480

    # Assume H1 is the "origin" hydrophone
    # If you change the values here, you must also change the values in the Unity editor
    # H1 is at the origin, H2 is on the x-axis, H3 is on the y-axis
    x = 0.05
    y = 0.05

    rospy.spin()

    

    