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
    q_ESD_imunominaldown_x = msg.orientation.x
    q_ESD_imunominaldown_y = msg.orientation.y
    q_ESD_imunominaldown_z = msg.orientation.z
    q_ESD_imunominaldown_w = msg.orientation.w

    q_ESD_imunominaldown = np.quaternion(
        q_ESD_imunominaldown_w, q_ESD_imunominaldown_x, q_ESD_imunominaldown_y, q_ESD_imunominaldown_z
    )

    q_ENU_imunominalup = q_ENU_ESD * q_ESD_imunominaldown * q_imunominaldown_imunominalup

    twist_angular_e = msg.angular_velocity.z
    twist_angular_n = msg.angular_velocity.x
    twist_angular_u = -msg.angular_velocity.y
    twist_enu = [-twist_angular_e,twist_angular_n,twist_angular_u]


    isDVLActive = msg.isDVLActive
    isIMUActive = msg.isIMUActive
    isDepthSensorActive = msg.isDepthSensorActive

    velocity_enu = [msg.velocity.z, -msg.velocity.x, msg.velocity.y]

    acceleration_enu = [msg.linear_acceleration.z, -msg.linear_acceleration.x,msg.linear_acceleration.y]


    # DVL - NWU
    if isDVLActive:

        q_ENU_dvlup =  q_ENU_imunominalup * q_imunominalup_dvlnominalup * q_dvlnominalup_dvlup

        velocity_dvl = quaternion.rotate_vectors(q_ENU_dvlup.inverse(), velocity_enu)

        dvl_msg = TwistWithCovariance()
        dvl_msg.twist.linear = Vector3(*velocity_dvl)

        pub_dvl_sensor.publish(dvl_msg)


    # IMU - NED
    if isIMUActive:
        imu_msg = Imu()

        q_ENU_imuup = q_ENU_imunominalup * q_imunominalup_imuup

        imu_msg.orientation = Quaternion(
            x=q_ENU_imuup.x, y=q_ENU_imuup.y, z=q_ENU_imuup.z, w=q_ENU_imuup.w
        )


        twist_imu = quaternion.rotate_vectors(
            q_ENU_imuup.inverse(), twist_enu
        )

        acceleration_imu = quaternion.rotate_vectors(
            q_ENU_imuup.inverse(), acceleration_enu
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
    q_dvlnominalup_dvlup_w = rospy.get_param("q_dvlnominalup_dvlup_w")
    q_dvlnominalup_dvlup_x = rospy.get_param("q_dvlnominalup_dvlup_x")
    q_dvlnominalup_dvlup_y = rospy.get_param("q_dvlnominalup_dvlup_y")
    q_dvlnominalup_dvlup_z = rospy.get_param("q_dvlnominalup_dvlup_z")
    q_dvlnominalup_dvlup = np.quaternion(
        q_dvlnominalup_dvlup_w, q_dvlnominalup_dvlup_x, q_dvlnominalup_dvlup_y, q_dvlnominalup_dvlup_z
    )

    auv_dvl_offset_x = rospy.get_param("auv_dvl_offset_x")
    auv_dvl_offset_y = rospy.get_param("auv_dvl_offset_y")
    auv_dvl_offset_z = rospy.get_param("auv_dvl_offset_z")

    q_imunominalup_imuup_w = rospy.get_param("q_imunominalup_imuup_w")
    q_imunominalup_imuup_x = rospy.get_param("q_imunominalup_imuup_x")
    q_imunominalup_imuup_y = rospy.get_param("q_imunominalup_imuup_y")
    q_imunominalup_imuup_z = rospy.get_param("q_imunominalup_imuup_z")

    q_imunominalup_imuup = np.quaternion(
        q_imunominalup_imuup_w, q_imunominalup_imuup_x, q_imunominalup_imuup_y, q_imunominalup_imuup_z
    )

    q_imunominaldown_imunominalup = np.quaternion(0,1,0,0)
    q_imunominalup_dvlnominalup = np.quaternion(1,0,0,0)

    # REFERENCE FRAME DEFINITIONS
    q_ENU_ESD = np.quaternion(0, 1, 0, 0)


    q_imunominaldown_dvlnominalup = np.quaternion(0,1,0,0)


    pub_dvl_sensor = rospy.Publisher(
        "/sensors/dvl/twist", TwistWithCovariance, queue_size=1
    )
    pub_depth_sensor = rospy.Publisher("/sensors/depth/z", PoseWithCovariance, queue_size=1)
    pub_imu_sensor = rospy.Publisher(
        "/sensors/imu", Imu, queue_size=1
    )

    # Set up subscribers and publishers
    rospy.Subscriber("/unity/state", UnityState, cb_unity_state)

    rospy.spin()
