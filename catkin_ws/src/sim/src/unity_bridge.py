#!/usr/bin/env python3

import rospy
import numpy as np

from auv_msgs.msg import DeadReckonReport, UnityState
from geometry_msgs.msg import Quaternion, Vector3, TwistWithCovariance, PoseWithCovariance 
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Int32
from tf import transformations
import quaternion

DEG_PER_RAD = 180 / np.pi


def cb_unity_state(msg):
    pose_x = msg.position.z
    pose_y = -msg.position.x
    pose_z = msg.position.y
    q_NED_imunominalned_x = msg.orientation.x
    q_NED_imunominalned_y = msg.orientation.y
    q_NED_imunominalned_z = msg.orientation.z
    q_NED_imunominalned_w = msg.orientation.w

    q_NED_imunominalned = np.quaternion(
        q_NED_imunominalned_w, q_NED_imunominalned_x, q_NED_imunominalned_y, q_NED_imunominalned_z
    )

    q_ENU_imunominalenu = q_ENU_NED * q_ENU_imunominalned * q_imunominalned_imunominal_enu

    twist_angular_n = -msg.angular_velocity.z
    twist_angular_w = msg.angular_velocity.x
    twist_angular_u = -msg.angular_velocity.yaw
    twist_enu = [-twist_angular_w,twist_angular_n,twist_angular_u]

    isDVLActive = msg.isDVLActive
    isIMUActive = msg.isIMUActive
    isDepthSensorActive = msg.isDepthSensorActive

    velocity_nwu = [msg.velocity.x, msg.velocity.y, msg.velocity.z]

    acceleration_enu = [msg.linear_acceleration.x, msg.linear_acceleration.y,msg.linear_acceleration.z]

    # DVL - NWU
    if isDVLActive:

        q_NWU_dvlnwu = q_NWU_NED * q_NED_imunominalned * q_imunominalned_dvlnominalnwu * q_dvlnominalnwu_dvlnwu

        velocity_dvl = quaternion.rotate_vectors(q_NWU_dvlnwu.inverse(), velocity_nwu)

        dvl_msg = TwistWithCovariance()
        dvl_msg.twist.linear = Vector3(*velocity_dvl)

        pub_dvl_sensor.publish(dvl_msg)


    # IMU - NED
    if isIMUActive:
        imu_msg = Imu()

        q_ENU_imu = q_ENU_imunominalenu * q_imunominalenu_imu

        imu_msg.orientation = Quaternion(
            x=q_ENU_imu.x, y=q_ENU_imu.y, z=q_ENU_imu.z, w=q_ENU_imu.w
        )


        twist_imu = quaternion.rotate_vectors(
            q_ENU_imu.inverse(), twist_enu
        )

        acceleration_imu = quaternion.rotate_vectors(
            q_ENU_imu.inverse(), acceleration_imu
        )

        imu_msg.angular_velocity = Vector3(*twist_imu)
        imu_msg.linear_acceleration = Vector3(*acceleration_imu)
        
        pub_imu_sensor.publish(imu_msg)

    # DEPTH SENSOR
    if isDepthSensorActive:
        depth_msg = PoseWithCovariance() 
        depth_msg.pose.position.z = pose_z
        pub_depth_sensor.publish(depth_msg)


if __name__ == "__main__":
    rospy.init_node("unity_bridge")

    # Load parameters
    q_dvlnominal_dvl_w = rospy.get_param("q_dvlnominal_dvl_w")
    q_dvlnominal_dvl_x = rospy.get_param("q_dvlnominal_dvl_x")
    q_dvlnominal_dvl_y = rospy.get_param("q_dvlnominal_dvl_y")
    q_dvlnominal_dvl_z = rospy.get_param("q_dvlnominal_dvl_z")
    q_dvlnominal_dvl = np.quaternion(
        q_dvlnominal_dvl_w, q_dvlnominal_dvl_x, q_dvlnominal_dvl_y, q_dvlnominal_dvl_z
    )

    auv_dvl_offset_x = rospy.get_param("auv_dvl_offset_x")
    auv_dvl_offset_y = rospy.get_param("auv_dvl_offset_y")
    auv_dvl_offset_z = rospy.get_param("auv_dvl_offset_z")

    q_imunominal_imu_w = rospy.get_param("q_imunominal_imu_w")
    q_imunominal_imu_x = rospy.get_param("q_imunominal_imu_x")
    q_imunominal_imu_y = rospy.get_param("q_imunominal_imu_y")
    q_imunominal_imu_z = rospy.get_param("q_imunominal_imu_z")

    q_imunominal_imu = np.quaternion(
        q_imunominal_imu_w, q_imunominal_imu_x, q_imunominal_imu_y, q_imunominal_imu_z
    )

    # REFERENCE FRAME DEFINITIONS
    random_vector = np.random.rand(4)
    random_vector = random_vector / np.linalg.norm(random_vector)
    # q_NWU_dvlref = np.quaternion(random_vector[0],random_vector[1],random_vector[2],random_vector[3])
    q_NWU_dvlref = np.quaternion(0, 1, 0, 0)
    q_imunominal_auv = np.quaternion(0, 1, 0, 0)
    q_dvlnominal_auv = np.quaternion(0, 1, 0, 0)
    q_ENU_NED = np.quaternion(0, 0.707, 0.707, 0)
    q_NWU_NED = np.quaternion(0, 1, 0, 0)
    q_dvl_auv = q_dvlnominal_dvl.conjugate() * q_dvlnominal_auv

    # TODO: ADD DVL VELOCITY

    pub_dvl_sensor = rospy.Publisher(
        "/sensors/dvl/pose", DeadReckonReport, queue_size=1
    )
    pub_depth_sensor = rospy.Publisher("/sensors/depth/z", Float64, queue_size=1)
    pub_imu_quat_sensor = rospy.Publisher(
        "/sensors/imu", Imu, queue_size=1
    )

    # Set up subscribers and publishers
    rospy.Subscriber("/unity/state", UnityState, cb_unity_state)

    rospy.spin()
