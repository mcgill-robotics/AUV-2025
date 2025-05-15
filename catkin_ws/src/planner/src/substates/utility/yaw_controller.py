#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg      import Float64, Bool
from geometry_msgs.msg import Quaternion, Vector3, Twist
from tf.transformations import quaternion_from_euler, quaternion_multiply

class YawController:
    def __init__(self):
        self.yaw= None       
        self.last_error = None       

        rospy.Subscriber("/state/theta/z",Float64,lambda m: setattr(self, "yaw", m.data),queue_size=1)

        rospy.Subscriber("/controls/pid/quat/error",Float64,lambda m: setattr(self, "last_error", m.data),queue_size=1)

        self._pub_quat_enable   = rospy.Publisher("/controls/pid/quat/enable",Bool, queue_size=1)
        self._pub_quat_setpoint = rospy.Publisher("/controls/pid/quat/setpoint",Quaternion, queue_size=1)

        self._pub_x_enable = rospy.Publisher("/controls/pid/x/enable", Bool, queue_size=1)
        self._pub_y_enable = rospy.Publisher("/controls/pid/y/enable", Bool, queue_size=1)
        self._pub_z_enable = rospy.Publisher("/controls/pid/z/enable", Bool, queue_size=1)


    @staticmethod
    def _wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def run(self,delta_deg: float,tol_deg:float = 1.0,timeout_s: float = 13.0,step_deg:  float = 2.0):
        """
        Rotate in place by +delta_deg (CCW positive) **via a ramp**:
        - send small `step_deg` yaw increments to the quaternion PID
        - block until |error| < tol_deg or timeout_s expires
        """
        tol= math.radians(tol_deg)
        step_rad= math.radians(abs(step_deg)) * (1 if delta_deg >= 0 else -1)
        total_rad = math.radians(delta_deg)

        rate = rospy.Rate(30)
        t_start = rospy.Time.now()
        

        # 1) wait for first yaw sample --------------------------------------
        while self.yaw is None and not rospy.is_shutdown():
            if (rospy.Time.now() - t_start).to_sec() > timeout_s:
                rospy.logerr("YawController: no yaw feedback – aborting")
                return
            rate.sleep()

        # 2) disable XYZ PIDs, enable quaternion PID ------------------------
        for pub in (self._pub_x_enable, self._pub_y_enable, self._pub_z_enable):
            pub.publish(False)
        self._pub_quat_enable.publish(True)
        rospy.loginfo("YawController: ramping %.1f° turn (step %.1f°)", delta_deg, step_deg)

        sent_rad = 0.0
        # 3) RAMP: feed the PID small increments ----------------------------
        while abs(sent_rad - total_rad) > 1e-4 and not rospy.is_shutdown():
            incr = step_rad
            print("incr %.2f rad (%.2f°)" % (incr, math.degrees(incr)))

            # clamp last increment so we don't overshoot the target
            if abs(total_rad - sent_rad) < abs(step_rad):
                incr = total_rad - sent_rad
                print("clamping incr to %.2f rad" % incr)
            sent_rad += incr

            goal_yaw = self._wrap(self.yaw + incr)
            q = quaternion_from_euler(0, 0, goal_yaw)
            self._pub_quat_setpoint.publish(Quaternion(*q))
            print("sent %.2f rad (%.2f°) to quaternion PID" % (incr, math.degrees(incr)))
            print("goal yaw %.2f rad (%.2f°)" % (goal_yaw, math.degrees(goal_yaw)))
            print("sent yaw %.2f rad (%.2f°)" % (self.yaw, math.degrees(self.yaw)))
            rate.sleep()

        # 4) wait until error < tolerance -----------------------------------
        t_wait = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.last_error is not None:
                # convert quaternion-error scalar to angle error (rad):
                #   angle  = 2 * arccos(w)
                angle_err = abs(2.0 * math.acos(max(-1.0, min(1.0, self.last_error))))
                if angle_err < tol:
                    rospy.loginfo("YawController: done (angle error %.2f°)",math.degrees(angle_err))
                    break
            if (rospy.Time.now() - t_wait).to_sec() > timeout_s:
                if self.last_error is not None:
                    angle_err = 2.0 * math.acos(max(-1.0, min(1.0, self.last_error)))
                    err_deg   = math.degrees(angle_err)
                else:
                    err_deg = float('nan')
                rospy.logwarn("YawController: timeout waiting for convergence (err %.2f°)", err_deg)
                break
            rate.sleep()
        # 5) turn the quaternion PID back off -------------------------------
        self._pub_quat_enable.publish(False)
        err_deg = math.degrees(self.last_error) if self.last_error else float('nan')
        rospy.loginfo("YawController: done (final error %.2f°)", err_deg)

# ---------------------------------------------------------------------------
# if __name__ == "__main__":
#     rospy.init_node("yaw_turn_ramp")
#     ctrl = YawController()
#     try:
#         rospy.loginfo("Spinning 90° in place…")
#         ctrl.run(delta_deg=90.0, step_deg=1.0)
#     except rospy.ROSInterruptException:
#         pass
