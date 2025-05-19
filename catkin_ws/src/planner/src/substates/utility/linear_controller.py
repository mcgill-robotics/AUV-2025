#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt, sin, cos
from std_msgs.msg import Bool, Float64


class LinearController:

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0
        self.x_setpoint_pub = rospy.Publisher("/controls/pid/x/setpoint", Float64, queue_size=10)
        self.y_setpoint_pub = rospy.Publisher("/controls/pid/y/setpoint", Float64, queue_size=10)
        self.z_setpoint_pub = rospy.Publisher("/controls/pid/z/setpoint", Float64, queue_size=10)
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

    def moveDeltaLocal(self, delta_x, delta_y, delta_z=0.0, tolerance=0.05, timeout=30):
        rospy.sleep(1.0)  # Let odometry settle

        # Compute target in odom frame
        target_x = self.x + delta_x
        target_y = self.y + delta_y
        target_z = self.z + delta_z

        print(f"Target x: {target_x:.3f}, y: {target_y:.3f}, z: {target_z:.3f}")

        # --- Step 2: POSITION CONTROL using PIDs ---
        self.enable_pid("x", True)
        self.enable_pid("y", True)
        self.enable_pid("z", True)

        self.x_setpoint_pub.publish(target_x)
        self.y_setpoint_pub.publish(target_y)
        self.z_setpoint_pub.publish(target_z)
        print("PUBLSIH")

        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout and not rospy.is_shutdown():
            err_x = abs(self.x - target_x)
            err_y = abs(self.y - target_y)
            err_z = abs(self.z - target_z)

            rospy.loginfo(f"err_x: {err_x:.3f}, err_y: {err_y:.3f}, err_z: {err_z:.3f}")
            if err_x < tolerance and err_y < tolerance and err_z < tolerance:
                break
            rate.sleep()

        # Disable PIDs
        self.enable_pid("x", False)
        self.enable_pid("y", False)
        self.enable_pid("z", False)
