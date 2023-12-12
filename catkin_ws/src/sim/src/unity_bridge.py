#!/usr/bin/env python3

import rospy
import numpy as np

from auv_msgs.msg import DeadReckonReport, UnityState
from geometry_msgs.msg import Quaternion, Vector3
from sbg_driver.msg import SbgImuData, SbgEkfQuat
from std_msgs.msg import Float64
from tf import transformations
import quaternion

DEG_PER_RAD = (180 / np.pi)

def cb_unity_state(msg):
    pose_x = msg.position[0]
    pose_y = msg.position[1]
    pose_z = msg.position[2]
    pose_quat_x = msg.orientation.x
    pose_quat_y = msg.orientation.y
    pose_quat_z = msg.orientation.z
    pose_quat_w = msg.orientation.w

    twist_linear_x = msg.velocity[0]
    twist_linear_y = msg.velocity[1]
    twist_linear_z = msg.velocity[2]
    twist_angular_x = msg.angular_velocity[0]
    twist_angular_y = msg.angular_velocity[1]
    twist_angular_z = msg.angular_velocity[2]

    isDVLActive = msg.isDVLActive
    isIMUActive = msg.isIMUActive
    isDepthSensorActive = msg.isDepthSensorActive

    # DVL - NWU
    if isDVLActive:
        # Position
        position_NWU_auv = np.array([pose_x, pose_y, pose_z])
        position_auv_dvlref = quaternion.rotate_vectors(q_NWU_dvlref.inverse(), position_NWU_auv)
        dvl_offset_NWU = quaternion.rotate_vectors(q_NWU_dvlref, np.array([auv_dvl_offset_x, auv_dvl_offset_y, auv_dvl_offset_z]))
        position_dvl_dvlref = position_auv_dvlref + dvl_offset_NWU

        # Orientation
        q_NWU_auv = np.quaternion(pose_quat_w, pose_quat_x, pose_quat_y, pose_quat_z)
        q_dvlref_dvl = q_NWU_dvlref.inverse() * q_NWU_auv * q_dvl_auv.inverse()
        # euler_dvlref_auv = quaternion.as_euler_angles(q_dvlref_dvl)
        euler_dvlref_dvl = transformations.euler_from_quaternion([q_dvlref_dvl.x, q_dvlref_dvl.y, q_dvlref_dvl.z, q_dvlref_dvl.w])


        dvl_msg = DeadReckonReport()

        dvl_msg.x = position_dvl_dvlref[0]
        dvl_msg.y = position_dvl_dvlref[1]
        dvl_msg.z = position_dvl_dvlref[2]
        dvl_msg.std = 0.0
        dvl_msg.status = 1
        dvl_msg.roll = euler_dvlref_dvl[0] * DEG_PER_RAD
        dvl_msg.pitch = euler_dvlref_dvl[1] * DEG_PER_RAD
        dvl_msg.yaw = euler_dvlref_dvl[2] * DEG_PER_RAD
        pub_dvl_sensor.publish(dvl_msg)

    # IMU - NED
    if isIMUActive:
        sbg_quat_msg = SbgEkfQuat()
        sbg_quat_msg.quaternion = Quaternion(pose_quat_x, pose_quat_y, pose_quat_z, pose_quat_w)
        pub_imu_quat_sensor.publish(sbg_quat_msg)

        sbg_data_msg = SbgImuData()
        sbg_data_msg.gyro = Vector3(twist_angular_x[0], twist_angular_y[1], twist_angular_z[2])
        pub_imu_data_sensor.publish(sbg_data_msg)

    # DEPTH SENSOR
    if isDepthSensorActive:
        depth_msg = Float64()
        depth_msg.data = pose_z
        pub_depth_sensor.publish(depth_msg)

    



if __name__ == '__main__':
    rospy.init_node('unity_bridge')

    # Load parameters
    q_dvl_auv_w = rospy.get_param("~q_dvl_auv_w")
    q_dvl_auv_x = rospy.get_param("~q_dvl_auv_x")
    q_dvl_auv_y = rospy.get_param("~q_dvl_auv_y")
    q_dvl_auv_z = rospy.get_param("~q_dvl_auv_z")
    
    auv_dvl_offset_x = rospy.get_param("~auv_dvl_offset_x")
    auv_dvl_offset_y = rospy.get_param("~auv_dvl_offset_y")
    auv_dvl_offset_z = rospy.get_param("~auv_dvl_offset_z")
    
    q_imu_auv_w = rospy.get_param("~q_imu_auv_w")
    q_imu_auv_x = rospy.get_param("~q_imu_auv_x")
    q_imu_auv_y = rospy.get_param("~q_imu_auv_y")
    q_imu_auv_z = rospy.get_param("~q_imu_auv_z")

    # REFERENCE FRAME DEFINITIONS
    q_NED_NWU = np.quaternion(0, 1, 0, 0)
    q_NWU_dvlref = np.quaternion(0,1,0,0)

    q_imu_auv = np.quaternion(q_imu_auv_w, q_imu_auv_x, q_imu_auv_y, q_imu_auv_z)

    q_auv_gazeboImu = np.quaternion(0, -0.70710678, 0, 0.70710678)
    q_NWU_gazeboImuRef = np.quaternion(0, -0.70710678, 0, 0.70710678)
    q_gazeboImu_imu = q_auv_gazeboImu.inverse() * q_imu_auv.inverse()

    q_dvl_auv = np.quaternion(q_dvl_auv_w, q_dvl_auv_x, q_dvl_auv_y, q_dvl_auv_z)



    # Set up subscribers and publishers
    rospy.Subscriber('/unity/state', UnityState, cb_unity_state)

    # TODO: ADD DVL VELOCITY
    
    pub_dvl_sensor = rospy.Publisher('/sensors/dvl/pose', DeadReckonReport, queue_size=1)
    pub_depth_sensor = rospy.Publisher('/sensors/depth/z', Float64, queue_size=1)
    pub_imu_quat_sensor = rospy.Publisher('/sensors/imu/quaternion', SbgEkfQuat, queue_size=1)
    pub_imu_data_sensor = rospy.Publisher('/sensors/imu/angular_velocity', SbgImuData, queue_size=1)

    rospy.spin()