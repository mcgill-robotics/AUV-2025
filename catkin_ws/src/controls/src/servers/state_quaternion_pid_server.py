#!/usr/bin/env python3

import actionlib
import rospy

import numpy as np
import quaternion
from servers.base_server import BaseServer

from std_msgs.msg import Bool
from auv_msgs.msg import StateQuaternionAction
from geometry_msgs.msg import Quaternion

class StateQuaternionServer(BaseServer):
    def __init__(self):
        super().__init__()
        self.server = actionlib.SimpleActionServer(
            "/controls/server/state",
            StateQuaternionAction,
            execute_cb=self.callback,
            auto_start=False,
        )
        self.previous_goal_quat = None
        self.previous_goal_x = None
        self.previous_goal_y = None
        self.previous_goal_z = None
        self.goal_id = 0

        self.enable_quat_sub = rospy.Subscriber(
            "/controls/pid/quat/enable", Bool, self.quat_enable_cb
        )
        self.enable_x_sub = rospy.Subscriber(
            "/controls/pid/x/enable", Bool, self.x_enable_cb
        )
        self.enable_y_sub = rospy.Subscriber(
            "/controls/pid/y/enable", Bool, self.y_enable_cb
        )
        self.enable_z_sub = rospy.Subscriber(
            "/controls/pid/z/enable", Bool, self.z_enable_cb
        )

        self.server.start()

    def x_enable_cb(self, data):
        if data.data == False:
            self.previous_goal_x = None

    def y_enable_cb(self, data):
        if data.data == False:
            self.previous_goal_y = None

    def z_enable_cb(self, data):
        if data.data == False:
            self.previous_goal_z = None

    def quat_enable_cb(self, data):
        if data.data == False:
            self.previous_goal_quat = None

    def callback(self, goal):
        print("\n\nQuaternion Server got goal:\n", goal)
        self.goal_id += 1
        my_goal = self.goal_id
        self.cancelled = False
        self.goal = goal

        if self.pose is None:
            print("FAILURE, STATE SERVER DOES NOT HAVE A POSE")
            return

        # Extract target position & orientation
        goal_position = [
            goal.pose.position.x,
            goal.pose.position.y,
            goal.pose.position.z,
        ]
        goal_quat = np.quaternion(
            goal.pose.orientation.w,
            goal.pose.orientation.x,
            goal.pose.orientation.y,
            goal.pose.orientation.z,
        )

        # Apply local/global and displacement options
        if goal.local.data:
            goal_position = self.local_to_global(goal_position)
        if goal.displace.data:
            goal_position, goal_quat = self.get_goal_after_displace(
                goal_position, goal_quat
            )

        # X axis
        if goal.do_x.data:
            self.previous_goal_x = goal_position[0]
            self.pub_x_enable.publish(Bool(True))
            self.pub_surge.publish(0)
            self.pub_sway.publish(0)
            self.pub_x_pid.publish(goal_position[0])
        elif self.previous_goal_x is not None:      # Look for previous x goal if x is not found
            goal_position[0] = self.previous_goal_x
            self.pub_x_enable.publish(Bool(True))
            self.pub_surge.publish(0)
            self.pub_sway.publish(0)
            self.pub_x_pid.publish(goal_position[0])
        else:
            goal_position[0] = None

        # Y axis
        if goal.do_y.data:
            self.previous_goal_y = goal_position[1]
            self.pub_y_enable.publish(Bool(True))
            self.pub_surge.publish(0)
            self.pub_sway.publish(0)
            self.pub_y_pid.publish(goal_position[1])
        elif self.previous_goal_y is not None:      # Look for previous y goal is y is not found
            goal_position[1] = self.previous_goal_y
            self.pub_y_enable.publish(Bool(True))
            self.pub_surge.publish(0)
            self.pub_sway.publish(0)
            self.pub_y_pid.publish(goal_position[1])
        else:
            goal_position[1] = None

        # Z axis with safety limits
        if goal.do_z.data:
            self.pub_z_enable.publish(Bool(True))
            raw_z = goal_position[2]
            min_z = rospy.get_param("min_safe_goal_depth")
            max_z = rospy.get_param("max_safe_goal_depth")
            safe_z = max(min(raw_z, max_z), min_z)
            if safe_z != raw_z:
                print(
                    f"WARN: Goal changed from {raw_z}m to {safe_z}m for safety."
                )
            self.previous_goal_z = safe_z
            goal_position[2] = safe_z
            self.pub_heave.publish(0)
            self.pub_z_pid.publish(safe_z)
        elif self.previous_goal_z is not None:
            goal_position[2] = self.previous_goal_z
            self.pub_z_enable.publish(Bool(True))
            self.pub_heave.publish(0)
            self.pub_z_pid.publish(goal_position[2])
        else:
            goal_position[2] = None

        # Quaternion orientation
        if goal.do_quaternion.data:
            self.previous_goal_quat = goal_quat
            self.pub_quat_enable.publish(Bool(True))
            q_msg = Quaternion(
                w=goal_quat.w,
                x=goal_quat.x,
                y=goal_quat.y,
                z=goal_quat.z,
            )
            self.pub_quat_pid.publish(q_msg)
        elif self.previous_goal_quat is not None:
            pq = self.previous_goal_quat
            self.pub_quat_enable.publish(Bool(True))
            q_msg = Quaternion(
                w=pq.w, x=pq.x, y=pq.y, z=pq.z
            )
            self.pub_quat_pid.publish(q_msg)
        else:
            goal_quat = None

        # Wait for settle
        settle_time = rospy.get_param("time_to_settle")
        loop_dt = 1.0 / rospy.get_param("settle_check_rate")
        settled = False

        while (
            not settled
            and not self.cancelled
            and my_goal == self.goal_id
            and not rospy.is_shutdown()
        ):
            start_t = rospy.get_time()
            while (
                not self.cancelled
                and my_goal == self.goal_id
                and self.check_status(
                    goal_position,
                    goal_quat,
                    goal.do_x.data,
                    goal.do_y.data,
                    goal.do_z.data,
                    goal.do_quaternion.data,
                )
                and not rospy.is_shutdown()
            ):
                if rospy.get_time() - start_t > settle_time:
                    settled = True
                    print("settled")
                    break
                rospy.sleep(loop_dt)

        if not self.cancelled and my_goal == self.goal_id:
            self.server.set_succeeded()

    def local_to_global(self, goal_position):
        pivot_quat = (
            self.previous_goal_quat
            if self.previous_goal_quat is not None
            else self.body_quat
        )
        global_goal = (
            pivot_quat
            * np.quaternion(0, goal_position[0], goal_position[1], goal_position[2])
            * pivot_quat.inverse()
        )
        return [global_goal.x, global_goal.y, global_goal.z]

    def get_goal_after_displace(self, goal_position_delta, goal_quat_delta):
        pivot_x = (
            self.previous_goal_x
            if self.previous_goal_x is not None
            else self.pose.position.x
        )
        pivot_y = (
            self.previous_goal_y
            if self.previous_goal_y is not None
            else self.pose.position.y
        )
        pivot_z = (
            self.previous_goal_z
            if self.previous_goal_z is not None
            else self.pose.position.z
        )
        pivot_quat = (
            self.previous_goal_quat
            if self.previous_goal_quat is not None
            else self.body_quat
        )

        goal_position = [
            pivot_x + goal_position_delta[0],
            pivot_y + goal_position_delta[1],
            pivot_z + goal_position_delta[2],
        ]
        goal_quat = pivot_quat * goal_quat_delta
        return goal_position, goal_quat

    def check_status(self, goal_position, goal_quaternion, do_x, do_y, do_z, do_quat):

        tolerance_position = rospy.get_param("pid_positional_tolerance")
        tolerance_quat_w = rospy.get_param("pid_quaternion_w_tolerance")

        if goal_position[0] is not None:
            pos_x_error = self.calculatePosError(self.pose.position.x, goal_position[0])
            if abs(pos_x_error) > tolerance_position:
                return False
        if goal_position[1] is not None:
            pos_y_error = self.calculatePosError(self.pose.position.y, goal_position[1])
            if abs(pos_y_error) > tolerance_position:
                return False
        if goal_position[2] is not None:
            pos_z_error = self.calculatePosError(self.pose.position.z, goal_position[2])
            if abs(pos_z_error) > tolerance_position:
                return False
        if goal_quaternion is not None:
            quat_error = self.calculateQuatError(self.body_quat, goal_quaternion)
            if abs(quat_error.w) < tolerance_quat_w:
                return False

        return True

    def calculatePosError(self, pos1, pos2):
        return abs(pos1 - pos2)

    def calculateQuatError(self, q1, q2):
        return q1.inverse() * q2
