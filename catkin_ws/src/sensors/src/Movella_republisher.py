#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from message_filters import Subscriber, ApproximateTimeSynchronizer

def callback(imu_msg, free_acc_msg, pub):
    new_imu = Imu()
    new_imu.header.stamp = imu_msg.header.stamp  # preserve measurement time

    frame_override = rospy.get_param("~frame_id", "")
    new_imu.header.frame_id = frame_override if frame_override else imu_msg.header.frame_id

    new_imu.orientation = imu_msg.orientation
    new_imu.orientation_covariance = imu_msg.orientation_covariance
    new_imu.angular_velocity = imu_msg.angular_velocity
    new_imu.angular_velocity_covariance = imu_msg.angular_velocity_covariance

    new_imu.linear_acceleration = free_acc_msg.vector
    new_imu.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance

    dt = abs((imu_msg.header.stamp - free_acc_msg.header.stamp).to_sec())
    if dt > 0.03:
        rospy.logwarn_throttle(5, f"IMU vs FreeAccel timestamp diff: {dt*1000:.1f} ms")

    pub.publish(new_imu)

def main():
    rospy.init_node("movella_republisher")

    pub_imu = rospy.Publisher("sensors/imu/data", Imu, queue_size=10)

    sub_free_acc = Subscriber("filter/free_acceleration", Vector3Stamped)
    sub_imu = Subscriber("imu/data", Imu)

    ats = ApproximateTimeSynchronizer([sub_imu, sub_free_acc], queue_size=10, slop=0.04)
    ats.registerCallback(callback, pub_imu)

    rospy.spin()

if __name__ == '__main__':
    main()
