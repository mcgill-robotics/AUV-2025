#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import quaternion
from auv_msgs.msg import PingerBearing, PingerTimeDifference
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32

class HydrophonesBearingNode(Node):

    def __init__(self):
        super().__init__('hydrophones_bearing')

        # Parameters
        self.declare_parameter('hydrophones_dx', 0.0)
        self.declare_parameter('hydrophones_dy', 0.0)
        self.declare_parameter('hydrophones_dz', 0.0)
        self.declare_parameter('hydrophones_time_unit', 1.0)

        self.hydrophones_dx = self.get_parameter('hydrophones_dx').get_parameter_value().double_value
        self.hydrophones_dy = self.get_parameter('hydrophones_dy').get_parameter_value().double_value
        self.hydrophones_dz = self.get_parameter('hydrophones_dz').get_parameter_value().double_value
        self.time_unit = self.get_parameter('hydrophones_time_unit').get_parameter_value().double_value

        self.auv_position = [0.0, 0.0, 0.0]
        self.auv_rotation = quaternion.quaternion(1, 0, 0, 0)
        self.is_active = False

        # Subscribers
        self.create_subscription(
            Int32,
            '/sensors/hydrophones/status',
            self.cb_hydrophones_status,
            10
        )
        self.create_subscription(
            PingerTimeDifference,
            '/sensors/hydrophones/pinger_time_difference',
            self.cb_hydrophones_time_difference,
            10
        )
        self.create_subscription(
            Pose,
            '/state/pose',
            self.cb_pose,
            10
        )

        # Publishers
        self.pub_pinger_bearing = self.create_publisher(
            PingerBearing,
            '/sensors/hydrophones/pinger_bearing',
            10
        )

        # Speed of sound in water
        self.c = 1480

    def calculate_time_measurements(self, delta_time):
        distance = delta_time * self.c
        return distance

    def solve_bearing_vector(self, distance, is_three_hydrophones):
        position = np.array([self.hydrophones_dx, self.hydrophones_dy, 0] if is_three_hydrophones else [self.hydrophones_dx, self.hydrophones_dy, self.hydrophones_dz])
        with np.errstate(divide='ignore', invalid='ignore'):
            bearing_vector = np.where(position != 0, distance / position, 0)
        return bearing_vector

    def cb_hydrophones_time_difference(self, msg):
        if not self.is_active or msg.frequency == 0:
            return

        # Convert times received in 10e-7 to seconds
        absolute_times = np.array(msg.times) * self.time_unit

        # Calculate time differences between hydrophone 0 and others.
        dt_hydrophones = absolute_times - absolute_times[0]
        # Only take time differences between microphone 0 and others.
        dt_hydrophones = dt_hydrophones[1:]

        is_three_hydrophones = False
        if len(dt_hydrophones) == 2:
            dt_hydrophones = np.append(dt_hydrophones, 0)
            is_three_hydrophones = True

        measurements = self.calculate_time_measurements(dt_hydrophones)
        bearing_vector_local = self.solve_bearing_vector(measurements, is_three_hydrophones)
        bearing_vector_global = quaternion.rotate_vectors(
            self.auv_rotation,
            np.array(bearing_vector_local)
        )

        if is_three_hydrophones:
            bearing_vector_global[2] = 0

        pinger_bearing_msg = PingerBearing()
        pinger_bearing_msg.frequency = msg.frequency
        pinger_bearing_msg.pinger_bearing.x = bearing_vector_global[0]
        pinger_bearing_msg.pinger_bearing.y = bearing_vector_global[1]
        pinger_bearing_msg.pinger_bearing.z = bearing_vector_global[2]
        pinger_bearing_msg.state_x = self.auv_position[0]
        pinger_bearing_msg.state_y = self.auv_position[1]
        pinger_bearing_msg.state_z = self.auv_position[2]

        self.pub_pinger_bearing.publish(pinger_bearing_msg)

    def cb_hydrophones_status(self, msg):
        self.is_active = msg.data

    def cb_pose(self, msg):
        self.auv_position[0] = msg.position.x
        self.auv_position[1] = msg.position.y
        self.auv_position[2] = msg.position.z
        self.auv_rotation = quaternion.quaternion(
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        )

def main(args=None):
    rclpy.init(args=args)
    node = HydrophonesBearingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
