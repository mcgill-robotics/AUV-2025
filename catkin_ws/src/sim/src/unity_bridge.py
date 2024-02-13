#!/usr/bin/env python3

import rospy
import numpy as np

from auv_msgs.msg import DeadReckonReport, UnityState
from geometry_msgs.msg import Quaternion, Vector3
from sbg_driver.msg import SbgImuData, SbgEkfQuat
from std_msgs.msg import Float64, Bool
from tf import transformations
import quaternion

DEG_PER_RAD = (180 / np.pi)

def cb_unity_state(msg):
    pose_x = msg.position.z
    pose_y = -msg.position.x
    pose_z = msg.position.y
    q_NED_imunominal_x = msg.orientation.x
    q_NED_imunominal_y = msg.orientation.y
    q_NED_imunominal_z = msg.orientation.z
    q_NED_imunominal_w = msg.orientation.w

    q_NED_imunominal = np.quaternion(q_NED_imunominal_w, q_NED_imunominal_x, q_NED_imunominal_y, q_NED_imunominal_z)

    twist_angular_x = -msg.angular_velocity.z
    twist_angular_y = msg.angular_velocity.x
    twist_angular_z = -msg.angular_velocity.y

    isDVLActive = msg.isDVLActive
    isIMUActive = msg.isIMUActive
    isDepthSensorActive = msg.isDepthSensorActive


    # DVL - NWU
    if isDVLActive:
        pub_dvl_sensor_status.publish(Bool(True))
        q_NWU_auv = q_NWU_NED * q_NED_imunominal  * q_imunominal_auv
        # Position
        position_NWU_auv = np.array([pose_x, pose_y, pose_z])
        dvl_offset_NWU = quaternion.rotate_vectors(q_NWU_auv, np.array([auv_dvl_offset_x, auv_dvl_offset_y, auv_dvl_offset_z]))
        position_NWU_auv+= dvl_offset_NWU
        position_dvl_dvlref = quaternion.rotate_vectors(q_NWU_dvlref.inverse(), position_NWU_auv)

        # Orientation
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
        pub_imu_sensor_status.publish(Bool(True))
        sbg_quat_msg = SbgEkfQuat()

        q_NED_imu = q_NED_imunominal * q_imunominal_imu
        sbg_quat_msg.quaternion = Quaternion(x = q_NED_imu.x, y = q_NED_imu.y, z= q_NED_imu.z, w = q_NED_imu.w)
        pub_imu_quat_sensor.publish(sbg_quat_msg)

        twist_angular_x, twist_angular_y, twist_angular_z = quaternion.rotate_vectors(q_NWU_auv.inverse(), [twist_angular_x, twist_angular_y, twist_angular_z])
        sbg_data_msg = SbgImuData()
        sbg_data_msg.gyro = Vector3(twist_angular_x, twist_angular_y, twist_angular_z)
        pub_imu_data_sensor.publish(sbg_data_msg)

    # DEPTH SENSOR
    if isDepthSensorActive:
        pub_depth_sensor_status.publish(Bool(True))
        depth_msg = Float64()
        depth_msg.data = pose_z
        pub_depth_sensor.publish(depth_msg)

    



if __name__ == '__main__':
    rospy.init_node('unity_bridge')

    # Load parameters
    
    q_dvlnominal_dvl_w = rospy.get_param("q_dvlnominal_dvl_w")
    q_dvlnominal_dvl_x = rospy.get_param("q_dvlnominal_dvl_x")
    q_dvlnominal_dvl_y = rospy.get_param("q_dvlnominal_dvl_y")
    q_dvlnominal_dvl_z = rospy.get_param("q_dvlnominal_dvl_z")
    q_dvlnominal_dvl = np.quaternion(q_dvlnominal_dvl_w,q_dvlnominal_dvl_x,q_dvlnominal_dvl_y,q_dvlnominal_dvl_z)

    auv_dvl_offset_x = rospy.get_param("auv_dvl_offset_x")
    auv_dvl_offset_y = rospy.get_param("auv_dvl_offset_y")
    auv_dvl_offset_z = rospy.get_param("auv_dvl_offset_z")
    
    q_imunominal_imu_w = rospy.get_param("q_imunominal_imu_w")
    q_imunominal_imu_x = rospy.get_param("q_imunominal_imu_x")
    q_imunominal_imu_y = rospy.get_param("q_imunominal_imu_y")
    q_imunominal_imu_z = rospy.get_param("q_imunominal_imu_z")

    q_imunominal_imu = np.quaternion(q_imunominal_imu_w, q_imunominal_imu_x, q_imunominal_imu_y, q_imunominal_imu_z)

    # REFERENCE FRAME DEFINITIONS
    random_vector = np.random.rand(4)
    random_vector = random_vector / np.linalg.norm(random_vector)
    q_NWU_dvlref = np.quaternion(random_vector[0],random_vector[1],random_vector[2],random_vector[3])
    q_imunominal_auv = np.quaternion(0,1,0,0)
    q_dvlnominal_auv = np.quaternion(0,1,0,0)
    q_NWU_NED = np.quaternion(0,1,0,0)
    q_dvl_auv = q_dvlnominal_dvl.conjugate() * q_dvlnominal_auv 

    # TODO: ADD DVL VELOCITY
    
    pub_imu_sensor_status = rospy.Publisher("/sensors/imu/status", Bool, queue_size=1)
    pub_depth_sensor_status = rospy.Publisher("/sensors/depth/status", Bool, queue_size=1)
    pub_dvl_sensor_status = rospy.Publisher("/sensors/dvl/status", Bool, queue_size=1)

    pub_dvl_sensor = rospy.Publisher('/sensors/dvl/pose', DeadReckonReport, queue_size=1)
    pub_depth_sensor = rospy.Publisher('/sensors/depth/z', Float64, queue_size=1)
    pub_imu_quat_sensor = rospy.Publisher('/sensors/imu/quaternion', SbgEkfQuat, queue_size=1)
    pub_imu_data_sensor = rospy.Publisher('/sensors/imu/angular_velocity', SbgImuData, queue_size=1)
    
    # Set up subscribers and publishers
    rospy.Subscriber('/unity/state', UnityState, cb_unity_state)

    rospy.spin()