#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Twist, Wrench
from math import atan2, sqrt, sin, cos
from std_msgs.msg import Bool, Float64
from auv_msgs.msg import (
    ThrusterMicroseconds,
)
class LinearController:

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.x_setpoint_pub = rospy.Publisher("/controls/pid/x/setpoint", Float64, queue_size=10)
        self.y_setpoint_pub = rospy.Publisher("/controls/pid/y/setpoint", Float64, queue_size=10)
        # Subscribe to odometry updates
        rospy.Subscriber("/odometry/filtered", Odometry, self.newOdom)
        # Publisher for movement commands
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def newOdom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

    def enable_pid(self, axis, state):
        pub = rospy.Publisher(f"/controls/pid/{axis}/enable", Bool, queue_size=1)
        pub.publish(Bool(state))

    
    def kill(self):
        pub_surge = rospy.Publisher("/controls/force/surge", Float64, queue_size=1)
        pub_sway = rospy.Publisher("/controls/force/sway", Float64, queue_size=1)
        pub_heave = rospy.Publisher("/controls/force/heave", Float64, queue_size=1)
        pub_roll = rospy.Publisher("/controls/torque/roll", Float64, queue_size=1)
        pub_pitch = rospy.Publisher("/controls/torque/pitch", Float64, queue_size=1)
        pub_yaw = rospy.Publisher("/controls/torque/yaw", Float64, queue_size=1)
        pub_global_x = rospy.Publisher("/controls/force/global/x", Float64, queue_size=1)
        pub_global_y = rospy.Publisher("/controls/force/global/y", Float64, queue_size=1)
        pub_global_z = rospy.Publisher("/controls/force/global/z", Float64, queue_size=1)
        pub_effort = rospy.Publisher("/controls/effort", Wrench, queue_size=1)
        pwm_pub = rospy.Publisher("/propulsion/microseconds", ThrusterMicroseconds, queue_size=1)

        # Disable all active PIDs
        self.enable_pid("x", False)
        self.enable_pid("y", False)
        self.enable_pid("z", False)
        self.enable_pid("quat", False)

        rospy.logwarn("Killing all thrusters and disabling PIDs...")

        start = rospy.get_time()
        while rospy.get_time() - start < 5:
            pub_surge.publish(0)
            pub_sway.publish(0)
            pub_heave.publish(0)
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

            # Send neutral PWM (1500) to all thrusters
            pwm_pub.publish(ThrusterMicroseconds([1500] * 8))

            rospy.sleep(0.1)


    def moveDeltaLocal(self, delta_x, delta_y, delta_z, tolerance=0.05, timeout=30):
        rospy.sleep(1.0)  # Let odometry settle

        # TODO: Make this map after SLAM
        # Compute target in odom frame
        target_x = self.x + delta_x
        target_y = self.y + delta_y

        print(f"Target x: {target_x:.3f}, y: {target_y:.3f}")

        # --- Step 2: POSITION CONTROL using PIDs ---
        self.enable_pid("x", True)
        self.enable_pid("y", True)

        self.x_setpoint_pub.publish(target_x)
        self.y_setpoint_pub.publish(target_y)
        # self.z_setpoint_pub.publish(target_z)

        rate = rospy.Rate(20)
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout and not rospy.is_shutdown():
            err_x = abs(self.x - target_x)
            err_y = abs(self.y - target_y)

            # rospy.loginfo(f"err_x: {err_x:.3f}, err_y: {err_y:.3f}")
            if err_x < tolerance and err_y < tolerance:
                break

            rate.sleep()

        # Disable PIDs
        # self.enable_pid("x", False)
        # self.enable_pid("y", False)
