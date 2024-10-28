#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from .ros_tcp_endpoint import TcpServer

import quaternion
import numpy as np

from tf import transformations
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import (
    Quaternion,
    Vector3,
    TwistWithCovarianceStamped,
    Pose,
    TransformStamped,
    PoseWithCovarianceStamped
)
from sensors_msgs.msg import Imu
from auv_msgs.msg import UnityState, PingerTimeDifference
from std_msgs.msg import Float64


DEG_PER_RAD = 180 / np.pi
NUMBER_OF_PINGERS = 4


class UnityBridge(Node):
    '''
    TODO: Document what the constructor does
    Constructs Node of class UnityBridge(Node)
    '''
    def __init__(self) -> None:
        super().__init__('unity_bridge')

        # Bypasses if Extended Kalman Filter not used
        self.bypass = not rclpy.get_parameter_value('ekf')
        self.reseted = False

        self.last_time = self.get_clock().now()

        self.tf_broadcaster = TransformBroadcaster(self)

        # Fetches parameters from either launch files or package declarations
        q_dvlnominalup_dvlup_w = self.get_parameter('q_dvlnominalup_dvlup_w').value
        q_dvlnominalup_dvlup_x = self.get_parameter('q_dvlnominalup_dvlup_x').value
        q_dvlnominalup_dvlup_y = self.get_parameter('q_dvlnominalup_dvlup_y').value
        q_dvlnominalup_dvlup_z = self.get_parameter('q_dvlnominalup_dvlup_z').value

        self.q_dvlnominalup_dvlup = np.quaternion(
            self.get_parameter('q_dvlnominalup_dvlup_w').value,
            self.get_parameter('q_dvlnominalup_dvlup_x').value,
            self.get_parameter('q_dvlnominalup_dvlup_y').value,
            self.get_parameter('q_dvlnominalup_dvlup_z').value
        )

        self.auv_dvl_offset_x = rclpy.get_parameter('auv_dvl_offset_x').value
        self.auv_dvl_offset_y = rclpy.get_parameter('auv_dvl_offset_y').value
        self.auv_dvl_offset_z = rclpy.get_parameter('auv_dvl_offset_z').value

        self.q_imunominalup_imuup = np.quaternion(
            rclpy.get_parameter('q_imunominalup_imuup_w').value,
            rclpy.get_parameter('q_imunominalup_imuup_x').value,
            rclpy.get_parameter('q_imunominalup_imuup_y').value,
            rclpy.get_parameter('q_imunominalup_imuup_z').value
        )

        self.q_imunominaldown_imunominalup = np.quaternion(0,1,0,0)
        self.q_imunominalup_dvlnominalup = np.quaternion(1,0,0,0)

        self.q_ENU_ESD = np.quaternion(0, 1, 0, 0)

        self.q_imunominaldown_dvlnominalup = np.quaternion(0, 1, 0, 0)

        # TODO: Add all publishers and subscribers for the node


    '''
    TODO: Document what reset_pose does.
    '''
    def reset_pose(self, pose) -> None:
        client = self.create_client(SetPose, 'set_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        msg = PoseWithCovarianceStamped()
        # TODO: Implement message details (head, pose), deal with exception,
        # and set_pose to new message

        return


    '''
    TODO: Document what publish_with_bypass does.
    '''
    def publish_with_bypass(self, pose, ang_vel) -> None:
        
        return


    '''
    TODO: Document what set_pose does.
    Helper method that creates Pose geometry message and initializes its Point(x, y, z) position and Quaternion(x, y, z, w) orientation fields.
    '''
    def set_pose(pose_x, pose_y, pose_z, q_ENU_imunominalup):
        pose = Pose()
        pose.position.x = pose_x
        pose.position.y = pose_y
        pose.position.z = pose_z

        quat = Quaternion(x=q_ENU_imunominalup.x, 
                          y=q_ENU_imunominalup.y, 
                          z=q_ENU_imunominalup.z, 
                          w=q_ENU_imunominalup.w)
        pose.orientation = quat
        
        return pose


    '''
    TODO: Document what unity_state_callback does
    '''
    def unity_state_callback(self, msg) -> None:
        pose_x = msg.position.z
        pose_y = -msg.position.x
        pose_z = msg.position.y
        
        q_ESD_imunominaldown_x = msg.orientation.x
        q_ESD_imunominaldown_y = msg.orientation.y
        q_ESD_imunominaldown_z = msg.orientation.z
        q_ESD_imunominaldown_w = msg.orientation.w
        q_ESD_imunominaldown = np.quaternion(q_ESD_imunominaldown_w, 
                                             q_ESD_imunominaldown_x, 
                                             q_ESD_imunominaldown_y, 
                                             q_ESD_imunominaldown_z)
        q_ENU_imunominalup = self.q_ENU_ESD * q_ESD_imunominaldown * self.q_imunominaldown_imunominalup

        twist_angular_e = msg.angular_velocity.z
        twist_angular_n = msg.angular_velocity.x
        twist_angular_u = -msg.angular_velocity.y
        twist_enu = [-twist_angular_e,twist_angular_n,twist_angular_u]

        frequencies = msg.frequencies
        times = [msg.times_pinger_1, msg.times_pinger_2, msg.times_pinger_3, msg.times_pinger_4]

        isDVLActive = msg.isDVLActive
        isIMUActive = msg.isIMUActive
        isDepthSensorActive = msg.isDepthSensorActive
        isHydrophonesActive = msg.isHydrophonesActive
        
        velocity_enu = [msg.velocity.z, -msg.velocity.x, msg.velocity.y]
        acceleration_enu = [msg.linear_acceleration.z, -msg.linear_acceleration.x, msg.linear_acceleration.y]

        # HYDROPHONES
        if isHydrophonesActive:
            for i in range(NUMBER_OF_PINGERS):
                hydrophones_msg = PingerTimeDifference()
                hydrophones_msg.frequency = frequencies[i]
                hydrophones_msg.times = times[i]
                # TODO: fix after publishers initialized in constructor
                pub_hydrophones_sensor.publish(hydrophones_msg)
           
        #  Scope shouldn't matter here but maybe watch this
        pose = self.set_pose(pose_x, pose_y, pose_z, q_ENU_imunominalup)
        
        if self.bypass:
            ang_vel_auv = quaternion.rotate_vectors(q_ENU_imunominalup.inverse(), 
                                                    twist_enu)
            ang_vel = Vector3(*ang_vel_auv)
            self.publish_with_bypass(pose, ang_vel)
            
            return
        
        if not self.reseted:
            self.reset_pose(pose)
            self.reseted = True
            
        # DVL - NWU
        if isDVLActive:
            q_ENU_dvlup =  q_ENU_imunominalup * self.q_imunominalup_dvlnominalup * self.q_dvlnominalup_dvlup

            velocity_dvl = quaternion.rotate_vectors(q_ENU_dvlup.inverse(), velocity_enu)

            dvl_msg = TwistWithCovarianceStamped()
            dvl_msg.twist.twist.linear = Vector3(*velocity_dvl)

            # ROS Time is deprecated, to get the time a node was initialized use rclpy.node.now()
            # Check when testing/debugging for same functionality.
            # Reference: https://docs.ros2.org/ardent/api/rclcpp/classrclcpp_1_1_node.html#af706b231620f1c120b7ccd738ec31867
            dvl_msg.header.stamp = self.now()
            dvl_msg.header.frame_id = "dvl"
                            
            # TODO: fix after publishers initialized in constructor
            pub_dvl_sensor.publish(dvl_msg)
            
        # IMU - NED
        if isIMUActive:
            imu_msg = Imu()

            q_ENU_imuup = q_ENU_imunominalup * self.q_imunominalup_imuup

            imu_msg.orientation = Quaternion(
                x=q_ENU_imuup.x, 
                y=q_ENU_imuup.y, 
                z=q_ENU_imuup.z, 
                w=q_ENU_imuup.w
            )

            twist_imu = quaternion.rotate_vectors(
                q_ENU_imuup.inverse(), 
                twist_enu
            )

            acceleration_imu = quaternion.rotate_vectors(
                q_ENU_imuup.inverse(), 
                acceleration_enu
            )

            imu_msg.angular_velocity = Vector3(*twist_imu)
            imu_msg.linear_acceleration = Vector3(*acceleration_imu)
            
            # ROS Time is deprecated, to get the time a node was initialized use rclpy.node.now()
            # Check when testing/debugging for same functionality.
            # Reference: https://docs.ros2.org/ardent/api/rclcpp/classrclcpp_1_1_node.html#af706b231620f1c120b7ccd738ec31867
            imu_msg.header.stamp = self.now()
            imu_msg.header.frame_id = "imu"

            # TODO: fix after publishers initialized in constructor
            pub_imu_sensor.publish(imu_msg)
            
        # DEPTH SENSOR
        if isDepthSensorActive:
            depth_msg = Float64() 
            depth_msg.data = pose_z

            # TODO: fix after publishers initialized in constructor
            pub_depth_sensor.publish(depth_msg)


def main(args=None):
    # Start the server endpoint
    rclpy.init(args=args)
    unity_bridge_node = UnityBridge()

    # Begin the TCP to Unity server and spin
    tcp_server = TcpServer(rclpy.get_name())
    tcp_server.start()
    rclpy.spin(unity_bridge_node)

    # Destroy and Shutdown the node
    unity_bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
