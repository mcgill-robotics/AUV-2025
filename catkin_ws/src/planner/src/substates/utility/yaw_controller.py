#!/usr/bin/env python3

import sys
import math
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class YawRamp:
    def __init__(self):
        self.yaw = None
        #sbscribe to EKF‐filtered odometry for a smooth yaw estimate
        rospy.Subscriber("/odometry/filtered", Odometry, self._odom_cb, queue_size=1)

        # Publishers for PID axis enables and quaternion setpoint
        self.pub_q_enable = rospy.Publisher("/controls/pid/quat/enable", Bool, queue_size=1)
        self.pub_q_setpoint = rospy.Publisher("/controls/pid/quat/setpoint", Quaternion, queue_size=1)
        for axis in "xyz":
            setattr(self,
                    f"pub_{axis}_enable",
                    rospy.Publisher(f"/controls/pid/{axis}/enable", Bool, queue_size=1))

    def _odom_cb(self, msg):
        # Extract yaw from odometry quaternion
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = yaw

    @staticmethod
    def _wrap(angle):
        # Wrap angle to (-pi, pi]
        return math.atan2(math.sin(angle), math.cos(angle))

    def run(self, delta_deg: float, step_deg: float = 3, tol_deg: float = 1.0, rate_hz: int = 200, timeout_s: float = 40.0):

        # Compute tolerance in radians
        tol_rad = math.radians(tol_deg)
        max_step_rad = math.radians(step_deg)
        rate = rospy.Rate(rate_hz)
        t0 = rospy.Time.now()

        # 1) Wait for first yaw reading
        while self.yaw is None and not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > timeout_s:
                rospy.logerr("YawRamp: no odometry - aborting")
                return
            rate.sleep()

        # Determine targets
        yaw_start = self.yaw
        yaw_target = self._wrap(yaw_start + math.radians(delta_deg))
        rospy.loginfo(
            "YawRamp: start %.2f°, target %.2f° (Δ %.1f°)",
            math.degrees(yaw_start), math.degrees(yaw_target), delta_deg
        )
        print(f"[DEBUG] Starting yaw: {math.degrees(yaw_start):.2f}°, Target yaw: {math.degrees(yaw_target):.2f}°")

        # Disable x/y/z PIDs and enable quaternion PID
        for pub in (self.pub_x_enable, self.pub_y_enable, self.pub_z_enable):
            pub.publish(False)
        self.pub_q_enable.publish(True)

        #use actual current yaw each iteration
        while not rospy.is_shutdown():
            curr = self.yaw
            err = self._wrap(yaw_target - curr)
            print(f"[DEBUG] Current yaw: {math.degrees(curr):.2f}°, Error to target: {math.degrees(err):.2f}°")

            #check if within tolerance
            if abs(err) < tol_rad:
                rospy.loginfo("YawRamp: reached target (|err| %.2f°)", math.degrees(err))
                print(f"[DEBUG] Final yaw within tol: {math.degrees(curr):.2f}°")
                break

            # Determine step direction based on error sign
            step = max_step_rad if abs(err) > max_step_rad else abs(err)
            step_signed = step if err >= 0 else -step

            #only move if step_signed actually moves toward target
            new_setpoint = self._wrap(curr + step_signed)
            #debug whether moving closer
            prev_dist = abs(self._wrap(yaw_target - curr))
            new_dist = abs(self._wrap(yaw_target - new_setpoint))
            if new_dist < prev_dist:
                print(f"[DEBUG] Advancing setpoint by {math.degrees(step_signed):.2f}° to {math.degrees(new_setpoint):.2f}°")
                q = quaternion_from_euler(0, 0, new_setpoint)
                self.pub_q_setpoint.publish(Quaternion(*q))
            else:
                print(f"[DEBUG] Step would increase error ({math.degrees(prev_dist):.2f}°→{math.degrees(new_dist):.2f}°), skipping move")

            rate.sleep()
            #timeout check
            if (rospy.Time.now() - t0).to_sec() > timeout_s:
                rospy.logwarn("YawRamp: timeout, err ≈ %.2f°", math.degrees(err))
                print(f"[DEBUG] Timeout at yaw {math.degrees(curr):.2f}°, err {math.degrees(err):.2f}°")
                break

        self.pub_q_enable.publish(False)
        rospy.loginfo("YawRamp: done")

