#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Float64

class DepthController:
    def __init__(self):
        self.z = 0.0
        # publishers
        self.z_setpoint_pub = rospy.Publisher("/controls/pid/z/setpoint", Float64, queue_size=20)
        self.pid_enable_pub = rospy.Publisher("/controls/pid/z/enable", Bool, queue_size=1)
        # subscriber for current depth
        rospy.Subscriber("/state/z", Float64, self.newOdom)

    def newOdom(self, msg):
        self.z = msg.data

    def enable_pid(self, state: bool):
        self.pid_enable_pub.publish(Bool(state))

    def submergeBy(self, delta_z, tolerance=0.05, timeout=35.0):
        rospy.sleep(2.0)

        # compute final target
        target_z = self.z + delta_z
        rospy.loginfo(f"depth submergeBy: current={self.z:.2f} -> target={target_z:.2f}")

        self.enable_pid(True)
        self.z_setpoint_pub.publish(Float64(target_z))

        rate = rospy.Rate(20)
        start = rospy.Time.now()

        #dive until within tolerance (or timeout)
        while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < timeout:
            err_z = abs(self.z - target_z)
            rospy.loginfo_throttle(1, f"depth error = {err_z:.3f}")
            if err_z < tolerance:
                rospy.loginfo("depth reached target depth")
                break
            rate.sleep()

        rospy.loginfo(f"[depth] holding at {target_z:.2f} m to fight buoyancy")
        while not rospy.is_shutdown():
            self.z_setpoint_pub.publish(Float64(target_z))
            rate.sleep()
