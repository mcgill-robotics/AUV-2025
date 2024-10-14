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
    TODO: Document what unity_state_callback does
    '''
    def unity_state_callback(self, msg) -> None:

        return

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
