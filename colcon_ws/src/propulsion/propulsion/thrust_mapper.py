#!/usr/bin/env python

"""

Description: Thrust mapper node subscribes to effort topic, converts the wrench readings to forces, 
and then finally converts the forces to pwm signals and publishes them.

"""

import numpy as np
import rclpy
from rclpy.node import Node
from thrust_mapper_utils import *
from auv_msgs.msg import ThrusterForces, ThrusterMicroseconds
from geometry_msgs.msg import Wrench
import time

class ThrustMapper(Node):
    def __init__(self):
        super().__init__("thrust_mapper")
        # Q: double or float?
        # constant parameters of the thruster positions
        self.l = self.get_parameter("distance_thruster_thruster_length").get_parameter_value().double_value
        self.w = self.get_parameter("distance_thruster_thruster_width").get_parameter_value().double_value
        self.alpha = self.get_parameter("angle_thruster").get_parameter_value().double_value
        self.a = self.get_parameter("distance_thruster_middle_length").get_parameter_value().double_value

        # Q: double or float?
        # used in the forces_to_pwm_publisher
        self.thruster_lower_limit = self.get_parameter("thruster_lower_limit").get_parameter_value().double_value
        self.thruster_upper_limit = self.get_parameter("thruster_upper_limit").get_parameter_value().double_value

        self.T_inv = self.inverse_transformation_matrix()  

        self.pub_us = self.create_publisher(ThrusterMicroseconds, "/propulsion/microseconds", 1)
        self.pub_forces = self.create_publisher(ThrusterForces, "/propulsion/forces", 1)
        self.create_subscription(Wrench, "/controls/effort", self.wrench_to_thrust, 1)
        self.on_shutdown(self.shutdown)
        self.re_arm()

        # Q: where should this be placed exactly?
        rclpy.sleep(7.0)  # TODO: FIX - wait for 7 sec to sync with arduino?

    def inverse_transformation_matrix(self):
        T = np.array(
            [
                [np.cos(self.alpha), 0, 0, -np.cos(self.alpha), -np.cos(self.alpha), 0, 0, np.cos(self.alpha)],
                [-np.sin(self.alpha), 0, 0, -np.sin(self.alpha), np.sin(self.alpha), 0, 0, np.sin(self.alpha)],
                [0, -1, -1, 0, 0, -1, -1, 0],
                [0, self.w / 2, self.w / 2, 0, 0, -self.w / 2, -self.w / 2, 0],
                [0, -self.a, self.a, 0, 0, self.a, -self.a, 0],
                [
                    self.w / 2 * np.cos(self.alpha) + self.l / 2 * np.sin(self.alpha),
                    0,
                    0,
                    -self.w / 2 * np.cos(self.alpha) - self.l / 2 * np.sin(self.alpha),
                    self.w / 2 * np.cos(self.alpha) + self.l / 2 * np.sin(self.alpha),
                    0,
                    0,
                    -self.w / 2 * np.cos(self.alpha) - self.l / 2 * np.sin(self.alpha),
                ],
            ]
        )

        # Matrix representation of the system of equations representing the thrust to wrench conversion
        # Ex: Force_X = (1)BACK_LEFT_Thruster + (1)HEAVE_BACK_LEFTboard_Thrust

        # matrix transformation wrench -> thrust
        T_inv = np.linalg.pinv(T)
        return T_inv

    def wrench_to_thrust(self, w):
        """
        A callback function that maps a Wrench into a force produced by T200 thruster at 14V (N)
        """
        a = np.array(
            [
                [w.force.x],
                [w.force.y],
                [w.force.z],
                [w.torque.x],
                [w.torque.y],
                [w.torque.z],
            ]
        )
        
        converted_w = np.matmul(self.T_inv, a)
        tf = ThrusterForces()

        tf.BACK_LEFT = converted_w[0][0]
        tf.HEAVE_BACK_LEFT = converted_w[1][0]
        tf.HEAVE_FRONT_LEFT = converted_w[2][0]
        tf.FRONT_LEFT = converted_w[3][0]
        tf.FRONT_RIGHT = converted_w[4][0]
        tf.HEAVE_FRONT_RIGHT = converted_w[5][0]
        tf.HEAVE_BACK_RIGHT = converted_w[6][0]
        tf.BACK_RIGHT = converted_w[7][0]

        # this is used by the sim
        self.pub_forces.publish(tf)

        # Convert forces to pwm signals and publish
        self.forces_to_pwm_publisher(tf)

    def forces_to_pwm_publisher(self, forces_msg):
        """
        Publish pwm signals
        """
        pwm_arr = [None] * 8
        pwm_arr[ThrusterMicroseconds.BACK_LEFT] = force_to_pwm(forces_msg.BACK_LEFT * thruster_mount_dirs[ThrusterMicroseconds.BACK_LEFT])
        pwm_arr[ThrusterMicroseconds.HEAVE_BACK_LEFT] = force_to_pwm(forces_msg.HEAVE_BACK_LEFT * thruster_mount_dirs[ThrusterMicroseconds.HEAVE_BACK_LEFT])
        pwm_arr[ThrusterMicroseconds.HEAVE_FRONT_LEFT] = force_to_pwm(forces_msg.HEAVE_FRONT_LEFT * thruster_mount_dirs[ThrusterMicroseconds.HEAVE_FRONT_LEFT])
        pwm_arr[ThrusterMicroseconds.FRONT_LEFT] = force_to_pwm(forces_msg.FRONT_LEFT * thruster_mount_dirs[ThrusterMicroseconds.FRONT_LEFT])
        pwm_arr[ThrusterMicroseconds.FRONT_RIGHT] = force_to_pwm(
            forces_msg.FRONT_RIGHT * thruster_mount_dirs[ThrusterMicroseconds.FRONT_RIGHT]
        )
        pwm_arr[ThrusterMicroseconds.HEAVE_FRONT_RIGHT] = force_to_pwm(
            forces_msg.HEAVE_FRONT_RIGHT * thruster_mount_dirs[ThrusterMicroseconds.HEAVE_FRONT_RIGHT]
        )
        pwm_arr[ThrusterMicroseconds.HEAVE_BACK_RIGHT] = force_to_pwm(
            forces_msg.HEAVE_BACK_RIGHT * thruster_mount_dirs[ThrusterMicroseconds.HEAVE_BACK_RIGHT]
        )
        pwm_arr[ThrusterMicroseconds.BACK_RIGHT] = force_to_pwm(
            forces_msg.BACK_RIGHT * thruster_mount_dirs[ThrusterMicroseconds.BACK_RIGHT]
        )

        # TODO - these are temporary precautionary measures and may result in unwanted dynamics
        # so as not to trip individual fuse (limit current draw)

        for i in range (len(pwm_arr)):
            if pwm_arr[i] > self.thrust_upper_limit:
                pwm_arr[i] = self.thrust_upper_limit
                self.get_logger().warn(f'INDIVIDUAL FUSE EXCEEDED: T{i + 1}')
            elif pwm_arr[i] < self.thrust_lower_limit:
                pwm_arr[i] = self.thrust_lower_limit
                self.get_logger().warn(f'INDIVIDUAL FUSE EXCEEDED: T{i + 1}')


        pwm_msg = ThrusterMicroseconds(pwm_arr)
        self.pub_us.publish(pwm_msg)

    # turns off the thursters when the node dies
    def shutdown(self):
        msg = ThrusterMicroseconds([1500] * 8)
        self.pub_us.publish(msg)

    # sends the arming signal to the thursters upon startup
    def re_arm(self):
        time.sleep(1)
        msg1 = ThrusterMicroseconds([1500] * 8)
        msg2 = ThrusterMicroseconds([1540] * 8)

        self.pub_us.publish(msg1)
        time.sleep(0.5)
        self.pub_us.publish(msg2)
        time.sleep(0.5)
        self.pub_us.publish(msg1)


def main(args=None):
    rclpy.init(args=args)
    thrust_mapper = ThrustMapper()

    rclpy.spin(thrust_mapper)

    # Destroy and Shutdown the node
    ThrustMapper.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()