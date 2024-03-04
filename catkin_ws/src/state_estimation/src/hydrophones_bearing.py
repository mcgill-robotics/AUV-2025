#!/usr/bin/env python3

import rospy

import numpy as np
from auv_msgs.msg import PingerBearing, PingerTimeDifference
from std_msgs.msg import Float64
from sbg_driver.msg import SbgEkfQuat
import quaternion


def cb_hydrophones_time_difference(msg):
    PingerBearing_msg = PingerBearing()

    if msg.is_pinger1_active:
        dt1_h1 = msg.dt_pinger1[0]
        dt1_h2 = msg.dt_pinger1[1]
        dt1_h3 = msg.dt_pinger1[2]
        measurements = calculate_time_measurements_3d(dt1_h1, dt1_h2, dt1_h3) 
        bearing_vector_local = solve_bearing_vector_3d(measurements[0], measurements[1], measurements[2])
        bearing_vector_global = quaternion.rotate_vectors(auv_rotation.inverse(), np.array([bearing_vector_local[0], bearing_vector_local[1], bearing_vector_local[2]]))
        PingerBearing_msg.pinger1_bearing.x = bearing_vector_global[0]
        PingerBearing_msg.pinger1_bearing.y = bearing_vector_global[1]
        PingerBearing_msg.pinger1_bearing.z = bearing_vector_global[2]

    if msg.is_pinger2_active:
        dt2_h1 = msg.dt_pinger2[0]
        dt2_h2 = msg.dt_pinger2[1]
        dt2_h3 = msg.dt_pinger2[2]
        measurements = calculate_time_measurements_3d(dt2_h1, dt2_h2, dt2_h3)
        bearing_vector_local = solve_bearing_vector_3d(measurements[0], measurements[1], measurements[2])
        bearing_vector_global = quaternion.rotate_vectors(auv_rotation.inverse(), np.array([bearing_vector_local[0], bearing_vector_local[1], bearing_vector_local[2]]))
        PingerBearing_msg.pinger2_bearing.x = bearing_vector_global[0]
        PingerBearing_msg.pinger2_bearing.y = bearing_vector_global[1]
        PingerBearing_msg.pinger2_bearing.z = bearing_vector_global[2]

    if msg.is_pinger3_active:
        dt3_h1 = msg.dt_pinger3[0]
        dt3_h2 = msg.dt_pinger3[1]
        dt3_h3 = msg.dt_pinger3[2]
        measurements = calculate_time_measurements_3d(dt3_h1, dt3_h2, dt3_h3)
        bearing_vector_local = solve_bearing_vector_3d(measurements[0], measurements[1], measurements[2])
        bearing_vector_global = quaternion.rotate_vectors(auv_rotation.inverse(), np.array([bearing_vector_local[0], bearing_vector_local[1], bearing_vector_local[2]]))
        PingerBearing_msg.pinger3_bearing.x = bearing_vector_global[0]
        PingerBearing_msg.pinger3_bearing.y = bearing_vector_global[1]
        PingerBearing_msg.pinger3_bearing.z = bearing_vector_global[2]

    if msg.is_pinger4_active:
        dt4_h1 = msg.dt_pinger4[0]
        dt4_h2 = msg.dt_pinger4[1]
        dt4_h3 = msg.dt_pinger4[2]
        measurements = calculate_time_measurements_3d(dt4_h1, dt4_h2, dt4_h3)
        bearing_vector_local = solve_bearing_vector_3d(measurements[0], measurements[1], measurements[2])
        bearing_vector_global = quaternion.rotate_vectors(auv_rotation.inverse(), np.array([bearing_vector_local[0], bearing_vector_local[1], 0]))
        PingerBearing_msg.pinger4_bearing.x = bearing_vector_global[0]
        PingerBearing_msg.pinger4_bearing.y = bearing_vector_global[1]
        PingerBearing_msg.pinger4_bearing.z = bearing_vector_global[2]

    PingerBearing_msg.state_x = state_x
    PingerBearing_msg.state_y = state_y
    PingerBearing_msg.state_z = state_z
    
    pub_pinger_bearing.publish(PingerBearing_msg)


def cb_imu_quat(msg):
    auv_rotation.w = msg.quaternion.w
    auv_rotation.x = msg.quaternion.x
    auv_rotation.y = msg.quaternion.y
    auv_rotation.z = msg.quaternion.z

def cb_state_x(msg):
    global state_x
    state_x = msg.data

def cb_state_y(msg):
    global state_y
    state_y = msg.data


#With 3 hydrophones, placed on x-y axes
def calculate_time_measurements(dt1, dt2):
    '''
    c = speed of sound in medium
    dn = distance from hydrophone 1 to hydrophone n
    '''
    d1 = c * dt1
    d2 = c * dt2
    return [d1,d2]

#With 4 hydrophones, placed on the x-y-z axes
def calculate_time_measurements_3d(dt1, dt2, dt3):
    '''
    c = speed of sound in medium
    dn = distance from hydrophone 1 to hydrophone n
    '''
    d1 = c * dt1
    d2 = c * dt2
    d3 = c * dt3
    return [d1,d2,d3]

def solve_bearing_vector(dx, dy):
    bearing_vector = [dx/x, dy/y]
    return bearing_vector

def solve_bearing_vector_3d(dx, dy, dz):
    bearing_vector = [dx/x, dy/y, dz/z]
    return bearing_vector

if __name__ == "__main__":
    rospy.init_node("hydrophones_bearing")
    rospy.Subscriber("/sensors/hydrophones/pinger_time_difference", PingerTimeDifference, cb_hydrophones_time_difference)
    rospy.Subscriber("/state/x", Float64, cb_state_x)
    rospy.Subscriber("/state/y", Float64, cb_state_y)
    rospy.Subscriber
    rospy.Subscriber("/sensors/imu/quaternion", SbgEkfQuat, cb_imu_quat)
    pub_pinger_bearing = rospy.Publisher("/sensors/hydrophones/pinger_bearing", PingerBearing, queue_size=1)

    auv_rotation = np.quaternion(1,0,0,0)   

    # Speed of sound in water
    c = 1480
    state_x = 0.0
    state_y = 0.0
    state_z = 0.0

    # Assume H1 is the "origin" hydrophone
    # If you change the values here, you must also change the values in the Unity editor
    # H1 is at the origin, H2 is on the x-axis, H3 is on the y-axis
    x = rospy.get_param("hydrophones_dx")
    y = rospy.get_param("hydrophones_dy")
    z = rospy.get_param("hydrophones_dz")

    rospy.spin()

    

    