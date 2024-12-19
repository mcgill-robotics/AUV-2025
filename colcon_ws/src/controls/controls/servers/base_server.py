#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Pose, Quaternion, Vector3
import numpy as np

"""
This class servers as an abstract class for the action lib servers the controls use to
execute goals. In the process of developing the controls servers, I found that the I was
duplicating a lot of code so I made this base class to avoid issues of fixing a bug in one
server but not fixing it in another.

The methods this class have are mainly boiler plate. There are helper methods to establish
publishers and subscribers, a default preempt callback that sets the pids to the current position,
methods to check if a goal pose has been entered, a method to automatically turn off pids,
and a method to publish setpoints to the pids.
"""


class BaseServer:
    def __init__(self, node) -> None:
        self.node_control = node
        self.cancelled = False
        self.goal = None
        self.pose = None
        self.body_quat = np.quaternion(1, 0, 0, 0)
        self.establish_effort_publishers()
        self.establish_pid_publishers()
        self.establish_pid_enable_publishers()
        self.establish_state_subscribers()

    def establish_effort_publishers(self):
        self.pub_surge = self.node_control.create_publisher(Float64, "/controls/force/surge", 1)
        self.pub_sway = self.node_control.create_publisher(Float64, "/controls/force/surge", 1)
        self.pub_heave = self.node_control.create_publisher(Float64, "/controls/force/heave", 1)
        self.pub_roll = self.node_control.create_publisher(Float64, "/controls/torque/roll", 1)
        self.pub_pitch = self.node_control.create_publisher(
            Float64, "/controls/torque/pitch", 1
        )
        self.pub_yaw = self.node_control.create_publisher(Float64, "/controls/torque/yaw", 1)
        self.pub_global_x = self.node_control.create_publisher(
            Float64, "/controls/force/global/x", 1
        )
        self.pub_global_y = self.node_control.create_publisher(
            Float64, "/controls/force/global/y", 1
        )
        self.pub_global_z = self.node_control.create_publisher(
            Float64, "/controls/force/global/z", 1
        )

    def establish_pid_publishers(self):
        self.pub_z_pid = self.node_control.create_publisher(
            Float64, "/controls/pid/z/setpoint", 1
        )
        self.pub_y_pid = self.node_control.create_publisher(
            Float64, "/controls/pid/y/setpoint", 1
        )
        self.pub_x_pid = self.node_control.create_publisher(
            Float64, "/controls/pid/x/setpoint", 1
        )
        self.pub_quat_pid = self.node_control.create_publisher(
            Quaternion, "/controls/pid/quat/setpoint", 1
        )

    def establish_pid_enable_publishers(self):
        self.pub_x_enable = self.node_control.create_publisher(
            "/controls/pid/x/enable", Bool, queue_size=1
        )
        self.pub_y_enable = self.node_control.create_publisher(
            "/controls/pid/y/enable", Bool, queue_size=1
        )
        self.pub_z_enable = self.node_control.create_publisher(
            "/controls/pid/z/enable", Bool, queue_size=1
        )
        self.pub_quat_enable = self.node_control.create_publisher(
            "/controls/pid/quat/enable", Bool, queue_size=1
        )

    def establish_state_subscribers(self):
        self.sub = self.node_control.create_subscription("/state/pose", Pose, self.set_pose)
        self.sub = self.node_control.create_subscription("/state/theta/x", Float64, self.set_theta_x)
        self.sub = self.node_control.create_subscription("/state/theta/y", Float64, self.set_theta_y)
        self.sub = self.node_control.create_subscription("/state/theta/z", Float64, self.set_theta_z)

    # callback for subscriber
    def set_pose(self, data):
        self.pose = data
        self.body_quat = np.quaternion(
            data.orientation.w,
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
        )
        if self.body_quat.w < 0:
            self.body_quat = -self.body_quat

    # callback for subscriber
    def set_theta_x(self, data):
        self.theta_x = data.data

    # callback for subscriber
    def set_theta_y(self, data):
        self.theta_y = data.data

    # callback for subscriber
    def set_theta_z(self, data):
        self.theta_z = data.data

    # generic cancel that publishes current position to pids to stay in place
    def cancel(self):
        self.cancelled = True

        self.pub_x_enable.publish(Bool(False))
        self.pub_y_enable.publish(Bool(False))
        self.pub_z_enable.publish(Bool(False))
        self.pub_quat_enable.publish(Bool(False))

        self.pub_global_x.publish(0)
        self.pub_global_y.publish(0)
        self.pub_global_z.publish(0)
        self.pub_roll.publish(0)
        self.pub_pitch.publish(0)
        self.pub_yaw.publish(0)
        self.pub_surge.publish(0)
        self.pub_sway.publish(0)
        self.pub_heave.publish(0)

        self.server.set_succeeded()
