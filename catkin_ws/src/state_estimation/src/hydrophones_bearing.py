#!/usr/bin/env python3

import rospy
import numpy as np
import quaternion

from auv_msgs.msg import PingerBearing, PingerTimeDifference
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32


def calculate_time_measurements(delta_time):
    distance = []
    for dt in delta_time: 
        distance.append(c * dt)
    return distance

def solve_bearing_vector(distance):
    position = [hydrophones_dx, hydrophones_dy] if len(distance) == 2 else [hydrophones_dx, hydrophones_dy, hydrophones_dz]
    bearing_vector = [distance[i] / position[i] if position[i] != 0 else position[i] for i in range(len(distance))]
    return bearing_vector


def cb_hydrophones_time_difference(msg):
    if not is_active or msg.frequency not in frequency_types:
        return  
     
    frequency_index = frequency_types.index(msg.frequency)     
    dt_hydrophones = [x * time_unit for x in msg.times]
    
    if len(dt_hydrophones) == 2:
        dt_hydrophones.append(0)
    
    measurements = calculate_time_measurements(dt_hydrophones)
    bearing_vector_local = solve_bearing_vector(measurements)
    bearing_vector_global = quaternion.rotate_vectors(
        auv_rotation,
        np.array(bearing_vector_local)
    )

    PingerBearing_msg = PingerBearing()
    PingerBearing_msg.frequency_index = frequency_index
    PingerBearing_msg.pinger_bearing.x = bearing_vector_global[0]
    PingerBearing_msg.pinger_bearing.y = bearing_vector_global[1]
    PingerBearing_msg.pinger_bearing.z = bearing_vector_global[2]
    PingerBearing_msg.state_x = auv_position[0]
    PingerBearing_msg.state_y = auv_position[1]
    PingerBearing_msg.state_z = auv_position[2]

    pub_pinger_bearing.publish(PingerBearing_msg)
   
def cb_hydrophones_status(msg):
    global is_active
    is_active = msg.data

def cb_pose(msg):
    global auv_position, auv_rotation
    auv_position[0] = msg.position.x
    auv_position[1] = msg.position.y
    auv_position[2] = msg.position.z
    auv_rotation = np.quaternion(
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
    )


if __name__ == "__main__":
    rospy.init_node("hydrophones_bearing")

    hydrophones_dx = rospy.get_param("hydrophones_dx")
    hydrophones_dy = rospy.get_param("hydrophones_dy")
    hydrophones_dz = rospy.get_param("hydrophones_dz")

    frequency_types = [
            rospy.get_param("pinger_frequency_1"), 
            rospy.get_param("pinger_frequency_2"), 
            rospy.get_param("pinger_frequency_3"), 
            rospy.get_param("pinger_frequency_4")
    ]

    time_unit = rospy.get_param("hydrophones_time_unit")

    auv_position = [0, 0, 0]
    auv_rotation = np.quaternion(1, 0, 0, 0)
    is_active = False

    rospy.Subscriber (
        "/sensors/hydrophones/status",
        Int32,
        cb_hydrophones_status
    )

    rospy.Subscriber(
        "/sensors/hydrophones/pinger_time_difference",
        PingerTimeDifference,
        cb_hydrophones_time_difference,
    )

    rospy.Subscriber("/state/pose", Pose, cb_pose)

    pub_pinger_bearing = rospy.Publisher(
        "/sensors/hydrophones/pinger_bearing", PingerBearing, queue_size=1
    )

    
    
    # Speed of sound in water
    c = 1480
    
    # Assume hx is the "origin" hydrophone
    # If you change the values here, you must also change the values in the Unity editor
    # hx is at the origin, hy is on the x-axis, hz is on the y-axis, H4 is on the z-axis
    

    rospy.spin()
