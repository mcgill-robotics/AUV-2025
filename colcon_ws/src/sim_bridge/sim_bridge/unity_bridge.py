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

from robot_localization.srv import SetPose

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
        self.q_dvlnominalup_dvlup = np.quaternion(
            self.declare_parameter('q_dvlnominalup_dvlup_w').value,
            self.declare_parameter('q_dvlnominalup_dvlup_x').value,
            self.declare_parameter('q_dvlnominalup_dvlup_y').value,
            self.declare_parameter('q_dvlnominalup_dvlup_z').value
        )

        self.declare_parameter('auv_dvl_offset_x')
        self.declare_parameter('auv_dvl_offset_y')
        self.declare_parameter('auv_dvl_offset_z')

        self.auv_dvl_offset_x = rclpy.get_parameter('auv_dvl_offset_x').value
        self.auv_dvl_offset_y = rclpy.get_parameter('auv_dvl_offset_y').value
        self.auv_dvl_offset_z = rclpy.get_parameter('auv_dvl_offset_z').value
        
        self.q_imunominalup_imuup = np.quaternion(
            self.declare_parameter('q_imunominalup_imuup_w').value,
            self.declare_parameter('q_imunominalup_imuup_x').value,
            self.declare_parameter('q_imunominalup_imuup_y').value,
            self.declare_parameter('q_imunominalup_imuup_z').value
        )

        self.q_imunominaldown_imunominalup = np.quaternion(0,1,0,0)
        self.q_imunominalup_dvlnominalup = np.quaternion(1,0,0,0)

        self.q_ENU_ESD = np.quaternion(0, 1, 0, 0)

        self.q_imunominaldown_dvlnominalup = np.quaternion(0, 1, 0, 0)

        self.create_publishers()
        self.create_subscriptions()

    '''
    TODO: Document what create_publishers does.
    '''
    def create_publishers(self) -> None:
        # Create Sensors Raw Data Publishers
        self.pub_dvl_sensor = self.create_publisher(TwistWithCovarianceStamped, '/sensors/dvl/Twist', 1)
        self.pub_depth_sensor = self.create_publisher(Float64, '/sensors/depth/z', 1)
        self.pub_imu_sensor = self.create_publisher(Imu, '/sensors/imu/data', 1)
        self.pub_hydrophone_sensor = self.create_publisher(PingerTimeDifference, '/sensors/hydrophones/pinger_time_difference', 1)

        # Create Coordinate Data Publishers
        self.pub_pose = self.create_publisher(Pose, '/state/post', 1)
        self.pub_x = self.create_publisher(Float64, '/state/x', 1)
        self.pub_y = self.create_publisher(Float64, '/state/y', 1)
        self.pub_z = self.create_publisher(Float64, '/state/z', 1)
        self.pub_theta_x = self.create_publisher(Float64, '/state/theta/x', 1)
        self.pub_theta_y = self.create_publisher(Float64, '/state/theta/y', 1)
        self.pub_theta_z = self.create_publisher(Float64, '/state/theta/z', 1)
        self.pub_ang_vel = self.create_publisher(Vector3, '/state/angular_velocity', 1)

    '''
    TODO: Document what create_subscription does.
    '''
    def create_subscriptions(self) -> None:
        self.sub_unity_state = self.create_subscription(UnityState, '/unity/state', unity_state_callback, rclpy.qos.QoSProfile())

    '''
    TODO: Document what reset_pose does.
    '''
    def reset_pose(self, pose) -> None:
        self.set_pose_cli = self.create_client(SetPose, 'set_pose')
        while not self.set_pose_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        request = SetPose.Request()
        request.msg = PoseWithCovarianceStamped()
        request.msg.pose.pose = pose
        request.msg.header.stamp = self.get_clock().now().to_msg()
        request.msg.header.frame_id = 'odom'

        future = self.set_pose_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        try:
            response = future.result()
            self.get_logger().info(f'Service call succeeded: {e}')
        except Exception as e:
            self.get_logger().info(f'Service call failed: {e}')


    '''
    TODO: Document what publish_with_bypass does.
    '''
    def publish_with_bypass(self, pose, ang_vel) -> None:
        self.pub_pose.publish(pose)
        self.pub_x.publish(pose.position.x)
        self.pub_y.publish(pose.position.y)
        self.pub_z.publish(pose.position.z)
        self.pub_ang_vel.publish(ang_vel)

        self.euler_dvlref_dvl = transformations.euler_from_quaternion(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )

        # roll = euler_dvlref_dvl[0] * DEG_PER_RAD
        # pitch = euler_dvlref_dvl[1] * DEG_PER_RAD
        # yaw = euler_dvlref_dvl[2] * DEG_PER_RAD

        self.pub_theta_x.publish(euler_dvlref_dvl[0] * DEG_PER_RAD)
        self.pub_theta_y.publish(euler_dvlref_dvl[1] * DEG_PER_RAD)
        self.pub_theta_z.publish(euler_dvlref_dvl[2] * DEG_PER_RAD)

        if rclpy.get_clock().now() == self.last_time:
            return
        self.last_time = rclpy.get_clock().now()

        # Generate message t and broadcast via sendTransform
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "auv_base"
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # Generate message t_rot and broadcast via sendTransform
        t_rot = TransformStamped()
        t_rot.header.stamp = rospy.Time.now()
        t_rot.header.frame_id = "world_rotation"
        t_rot.child_frame_id = "auv_rotation"
        t_rot.transform.translation.x = 0
        t_rot.transform.translation.y = 0
        t_rot.transform.translation.z = 0
        t_rot.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(t_rot)
        
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
            # Reference: https://answers.ros.org/question/321536/replacement-for-rospytimenow-in-ros2/
            dvl_msg.header.stamp = self.get_clock().now()
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
            # Reference: https://answers.ros.org/question/321536/replacement-for-rospytimenow-in-ros2/
            imu_msg.header.stamp = self.get_clock().now()
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
