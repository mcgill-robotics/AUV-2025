#!/usr/bin/env python3

import rospy

import numpy as np
from auv_msgs.msg import PingerBearing, PingerTimeDifference
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
import quaternion


def cb_hydrophones_time_difference(msg):
    PingerBearing_msg = PingerBearing()

    if (check_four_hydrophones(msg)):
        if msg.is_pinger1_active:
            dt_hx = msg.dt_pinger1[0]
            dt_hy = msg.dt_pinger1[1]
            dt_hz = msg.dt_pinger1[2]
            measurements = calculate_time_measurements_3d(dt_hx, dt_hy, dt_hz) 
            bearing_vector_local = solve_bearing_vector_3d(measurements[0], measurements[1], measurements[2])
            bearing_vector_global = quaternion.rotate_vectors(auv_rotation, np.array([bearing_vector_local[0], bearing_vector_local[1], bearing_vector_local[2]]))
            PingerBearing_msg.pinger1_bearing.x = bearing_vector_global[0]
            PingerBearing_msg.pinger1_bearing.y = bearing_vector_global[1]
            PingerBearing_msg.pinger1_bearing.z = bearing_vector_global[2]

        if msg.is_pinger2_active:
            dt_hx = msg.dt_pinger2[0]
            dt_hy = msg.dt_pinger2[1]
            dt_hz = msg.dt_pinger2[2]
            measurements = calculate_time_measurements_3d(dt_hx, dt_hy, dt_hz)
            bearing_vector_local = solve_bearing_vector_3d(measurements[0], measurements[1], measurements[2])
            bearing_vector_global = quaternion.rotate_vectors(auv_rotation, np.array([bearing_vector_local[0], bearing_vector_local[1], bearing_vector_local[2]]))
            PingerBearing_msg.pinger2_bearing.x = bearing_vector_global[0]
            PingerBearing_msg.pinger2_bearing.y = bearing_vector_global[1]
            PingerBearing_msg.pinger2_bearing.z = bearing_vector_global[2]

        if msg.is_pinger3_active:
            dt_hx = msg.dt_pinger3[0]
            dt_hy = msg.dt_pinger3[1]
            dt_hz = msg.dt_pinger3[2]
            measurements = calculate_time_measurements_3d(dt_hx, dt_hy, dt_hz)
            bearing_vector_local = solve_bearing_vector_3d(measurements[0], measurements[1], measurements[2])
            bearing_vector_global = quaternion.rotate_vectors(auv_rotation, np.array([bearing_vector_local[0], bearing_vector_local[1], bearing_vector_local[2]]))
            PingerBearing_msg.pinger3_bearing.x = bearing_vector_global[0]
            PingerBearing_msg.pinger3_bearing.y = bearing_vector_global[1]
            PingerBearing_msg.pinger3_bearing.z = bearing_vector_global[2]

        if msg.is_pinger4_active:
            dt_hx = msg.dt_pinger4[0]
            dt_hy = msg.dt_pinger4[1]
            dt_hz = msg.dt_pinger4[2]
            measurements = calculate_time_measurements_3d(dt_hx, dt_hy, dt_hz)
            bearing_vector_local = solve_bearing_vector_3d(measurements[0], measurements[1], measurements[2])
            bearing_vector_global = quaternion.rotate_vectors(auv_rotation, np.array([bearing_vector_local[0], bearing_vector_local[1], bearing_vector_local[2]]))
            PingerBearing_msg.pinger4_bearing.x = bearing_vector_global[0]
            PingerBearing_msg.pinger4_bearing.y = bearing_vector_global[1]
            PingerBearing_msg.pinger4_bearing.z = bearing_vector_global[2]
    else:
        if msg.is_pinger1_active:
            dt_hx = msg.dt_pinger1[0]
            dt_hy = msg.dt_pinger1[1]
            measurements = calculate_time_measurements(dt_hx, dt_hy) 
            bearing_vector_local = solve_bearing_vector(measurements[0], measurements[1])
            bearing_vector_global = quaternion.rotate_vectors(auv_rotation, np.array([bearing_vector_local[0], bearing_vector_local[1], 0]))
            PingerBearing_msg.pinger1_bearing.x = bearing_vector_global[0]
            PingerBearing_msg.pinger1_bearing.y = bearing_vector_global[1]
            PingerBearing_msg.pinger1_bearing.z = 0

        if msg.is_pinger2_active:
            dt_hx = msg.dt_pinger2[0]
            dt_hy = msg.dt_pinger2[1]
            measurements = calculate_time_measurements(dt_hx, dt_hy)
            bearing_vector_local = solve_bearing_vector(measurements[0], measurements[1])
            bearing_vector_global = quaternion.rotate_vectors(auv_rotation, np.array([bearing_vector_local[0], bearing_vector_local[1], 0]))
            PingerBearing_msg.pinger2_bearing.x = bearing_vector_global[0]
            PingerBearing_msg.pinger2_bearing.y = bearing_vector_global[1]
            PingerBearing_msg.pinger2_bearing.z = 0

        if msg.is_pinger3_active:
            dt_hx = msg.dt_pinger3[0]
            dt_hy = msg.dt_pinger3[1]
            measurements = calculate_time_measurements(dt_hx, dt_hy)
            bearing_vector_local = solve_bearing_vector(measurements[0], measurements[1])
            bearing_vector_global = quaternion.rotate_vectors(auv_rotation, np.array([bearing_vector_local[0], bearing_vector_local[1], 0]))
            PingerBearing_msg.pinger3_bearing.x = bearing_vector_global[0]
            PingerBearing_msg.pinger3_bearing.y = bearing_vector_global[1]
            PingerBearing_msg.pinger3_bearing.z = 0

        if msg.is_pinger4_active:
            dt_hx = msg.dt_pinger4[0]
            dt_hy = msg.dt_pinger4[1]
            measurements = calculate_time_measurements(dt_hx, dt_hy)
            bearing_vector_local = solve_bearing_vector(measurements[0], measurements[1])
            bearing_vector_global = quaternion.rotate_vectors(auv_rotation, np.array([bearing_vector_local[0], bearing_vector_local[1], 0]))
            PingerBearing_msg.pinger4_bearing.x = bearing_vector_global[0]
            PingerBearing_msg.pinger4_bearing.y = bearing_vector_global[1]
            PingerBearing_msg.pinger4_bearing.z = 0


    PingerBearing_msg.state_x = state_x
    PingerBearing_msg.state_y = state_y
    PingerBearing_msg.state_z = state_z
    
    pub_pinger_bearing.publish(PingerBearing_msg)

# Checks if there are four hydrophones data, returns boolean
def check_four_hydrophones(msg):
    if (len(msg.dt_pinger1) == 3 and len(msg.dt_pinger2) == 3 and len(msg.dt_pinger3) == 3 and len(msg.dt_pinger4) == 3):
        return True
    else:
        return False


def cb_quat(msg):
    global auv_rotation
    auv_rotation = np.quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)

def cb_state_x(msg):
    global state_x
    state_x = msg.data

def cb_state_y(msg):
    global state_y
    state_y = msg.data

def cb_state_z(msg):
    global state_z
    state_z = msg.data


#With 3 hydrophones, placed on x-y axes
def calculate_time_measurements(dtx, dty):
    '''
    c = speed of sound in medium
    dn = distance from hydrophone 1 to hydrophone n
    '''
    dx = c * dtx
    dy = c * dty
    return [dx,dy]

#With 4 hydrophones, placed on the x-y-z axes
def calculate_time_measurements_3d(dtx, dty, dtz):
    '''
    c = speed of sound in medium
    dn = distance from hydrophone 1 to hydrophone n
    '''
    dx = c * dtx
    dy = c * dty
    dz = c * dtz
    return [dx,dy,dz]

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
    rospy.Subscriber("/state/z", Float64, cb_state_z)
    rospy.Subscriber("/state/pose", Pose, cb_quat)
    pub_pinger_bearing = rospy.Publisher("/sensors/hydrophones/pinger_bearing", PingerBearing, queue_size=1)

    auv_rotation = np.quaternion(1,0,0,0)   

    # Speed of sound in water
    c = 1480
    state_x = 0.0
    state_y = 0.0
    state_z = 0.0

    # Assume hx is the "origin" hydrophone
    # If you change the values here, you must also change the values in the Unity editor
    # hx is at the origin, hy is on the x-axis, hz is on the y-axis, H4 is on the z-axis
    x = rospy.get_param("hydrophones_dx")
    y = rospy.get_param("hydrophones_dy")
    z = rospy.get_param("hydrophones_dz")

    rospy.spin()