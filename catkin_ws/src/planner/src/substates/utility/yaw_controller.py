#!/usr/bin/env python3
import math
import rospy

from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, Wrench
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler
from auv_msgs.msg import (
    ThrusterMicroseconds,
)

class YawRamp:
    def __init__(self):
        self.yaw = None
        # Subscribe to current yaw value (in radians)
        rospy.Subscriber("/state/theta/z", Float64, self.euler_cb, queue_size=15)

        # Publisher to enable quaternion PID controller
        self.pub_q_enable = rospy.Publisher("/controls/pid/quat/enable", Bool, queue_size=1)
        self.pub_q_setpoint = rospy.Publisher("/controls/pid/quat/setpoint", Quaternion, queue_size=1)

        # Publishers to disable x/y/z linear PID controllers
        self.pub_x_enable = rospy.Publisher("/controls/pid/x/enable", Bool, queue_size=1)
        self.pub_y_enable = rospy.Publisher("/controls/pid/y/enable", Bool, queue_size=1)
        self.pub_z_enable = rospy.Publisher("/controls/pid/z/enable", Bool, queue_size=1)

    def euler_cb(self, msg):
        self.yaw = msg.data
    
    def kill(self):
        rospy.logwarn("YawRamp: KILL sequence initiated – stopping all motion")

        pub_roll = rospy.Publisher("/controls/torque/roll", Float64, queue_size=1)
        pub_pitch = rospy.Publisher("/controls/torque/pitch", Float64, queue_size=1)
        pub_yaw = rospy.Publisher("/controls/torque/yaw", Float64, queue_size=1)

        pub_global_x = rospy.Publisher("/controls/force/global/x", Float64, queue_size=1)
        pub_global_y = rospy.Publisher("/controls/force/global/y", Float64, queue_size=1)
        pub_global_z = rospy.Publisher("/controls/force/global/z", Float64, queue_size=1)

        pub_effort = rospy.Publisher("/controls/effort", Wrench, queue_size=1)
        pwm_pub = rospy.Publisher("/propulsion/microseconds", ThrusterMicroseconds, queue_size=1)

        self.pub_x_enable.publish(False)
        self.pub_y_enable.publish(False)
        self.pub_z_enable.publish(False)
        self.pub_q_enable.publish(False)

        start = rospy.get_time()
        while rospy.get_time() - start < 5:
            pub_roll.publish(0)
            pub_pitch.publish(0)
            pub_yaw.publish(0)

            pub_global_x.publish(0)
            pub_global_y.publish(0)
            pub_global_z.publish(0)

            zero_wrench = Wrench()
            zero_wrench.force.x = 0
            zero_wrench.force.y = 0
            zero_wrench.force.z = 0
            zero_wrench.torque.x = 0
            zero_wrench.torque.y = 0
            zero_wrench.torque.z = 0
            pub_effort.publish(zero_wrench)

            pwm_pub.publish(ThrusterMicroseconds([1500] * 8))  #neutral signal

            rospy.sleep(0.1)

        rospy.loginfo("YawRamp: Kill sequence complete")
 
    
    @staticmethod
    def _wrap(angle):
        """Wraps angle to the (-pi, pi] range to avoid discontinuities"""
        return math.atan2(math.sin(angle), math.cos(angle))

    #here timeout is how much time youre giving the robot to get to the angle with error 0.5.
    def run(self, delta_deg: float, step_deg: float = 5, tol_deg: float = 0.5, rate_hz: int = 350, timeout_s: float = 40.0):

        tol_rad = math.radians(tol_deg)
        max_step_rad = math.radians(step_deg)
        rate = rospy.Rate(rate_hz)
        start_time = rospy.Time.now()

        while self.yaw is None and not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > timeout_s:
                rospy.logerr("YawRamp: No yaw data received - aborting")
                return
            rate.sleep()

        yaw_start = self.yaw
        yaw_target = self._wrap(yaw_start + math.radians(delta_deg))

        rospy.loginfo(
            "YawRamp: start = %.2f°, target = %.2f°, Δ = %.2f°",
            math.degrees(yaw_start), math.degrees(yaw_target), delta_deg
        )

        # Disable linear PID controllers
        self.pub_x_enable.publish(False)
        self.pub_y_enable.publish(False)
        self.pub_z_enable.publish(False)

        # Enable quaternion (angular) PID controller
        self.pub_q_enable.publish(True)

        #step-wise adjustment loop
        while not rospy.is_shutdown():
            curr_yaw = self.yaw
            error = self._wrap(yaw_target - curr_yaw)

            rospy.loginfo_throttle(1.0, "YawRamp Current: %.2f°, Error: %.2f°", math.degrees(curr_yaw), math.degrees(error))

            if abs(error) < tol_rad:
                rospy.loginfo("YawRamp: Reached target (error = %.2f°)", math.degrees(error))
                break

            #choose direction of rotation and size of step(smaller step = more accurate but longer wait time)
            step = min(max_step_rad, abs(error))
            step_signed = step if error >= 0 else -step
            new_setpoint_yaw = self._wrap(curr_yaw + step_signed)

            # Publish updated quaternion setpoint
            quat = quaternion_from_euler(0, 0, new_setpoint_yaw)
            self.pub_q_setpoint.publish(Quaternion(*quat))

            # Check for timeout
            if (rospy.Time.now() - start_time).to_sec() > timeout_s:
                rospy.logwarn("YawRamp: Timeout. Final error = %.2f°", math.degrees(error))
                break
            rate.sleep()

        self.pub_q_enable.publish(False)
        rospy.loginfo("YawRamp: Rotation complete.")
