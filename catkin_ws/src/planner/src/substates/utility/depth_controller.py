#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64

class DepthController:
    def __init__(self):
        self.z = 0.0

        # publishers
        self.z_setpoint_pub = rospy.Publisher("/controls/pid/z/setpoint", Float64, queue_size=1)
        self.pid_enable_pub  = rospy.Publisher("/controls/pid/z/enable",Bool,queue_size=1)

        # subscriber for current depth
        rospy.Subscriber("/state/z", Float64, self.newOdom)

    def newOdom(self, msg):
        self.z = msg.data

    def enable_pid(self, state: bool):
        self.pid_enable_pub.publish(Bool(state))

    def submergeBy(self, delta_z, tolerance=0.05, timeout=35.0):
        # give the PID node a moment to connect
        rospy.sleep(1.0)
        target_z = self.z + delta_z
        rospy.loginfo(f"[depth] submergeBy: current={self.z:.2f} → target={target_z:.2f}")
        # turn on the PID and send setpoint
        self.enable_pid(True)
        self.z_setpoint_pub.publish(Float64(target_z))
        rate = rospy.Rate(20)
        start = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < timeout:
            err_z = abs(self.z - target_z)
            rospy.loginfo_throttle(1, f"[depth] error = {err_z:.3f}")
            if err_z < tolerance:
                rospy.loginfo("[depth] tolerance reached, stopping.")
                break
            rate.sleep()