#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from message_filters import Subscriber, ApproximateTimeSynchronizer

<<<<<<< HEAD
THRESHOLD = 0.025

"""
Movella IMU Free Acceleration Republisher Node

This node subscribes to two topics published by a Movella (Xsens) IMU:
  - /imu/data (sensor_msgs/Imu): Contains orientation, angular velocity, and local acceleration.
  - /filter/free_acceleration (geometry_msgs/Vector3Stamped): Contains acceleration with gravity removed.

It uses an ApproximateTimeSynchronizer to match messages from both topics based on timestamps,
overwrites the linear_acceleration field of the IMU message with the free acceleration vector,
and republishes the modified IMU message to:
  - /sensors/imu/data (sensor_msgs/Imu)

Behavior:
  - Keeps the original IMU timestamp.
  - Explicitly remaps the frame_id to "imu".
  - Copies orientation and angular velocity data unchanged.
  - Warns if IMU and free acceleration timestamps differ by more than the configured threshold (default: 30 ms).

Intended use:
  - Provides a combined IMU message with gravity-compensated acceleration for downstream state estimation nodes.
  - Sampling rate expected at ~25 Hz for both topics.
"""

def callback(imu_msg, free_acc_msg, pub, threshold=THRESHOLD):
=======
def callback(imu_msg, free_acc_msg, pub):
>>>>>>> bb8abd67 (added debug statements in superimposer, and made the following changes to the republisher files. In depth republisher, we assign big cov to everything but z -> avoid confclits, also the type is float which has no head. for dvl cov we capture a stamp once per line and reuse it, we used to have seperate call())
    new_imu = Imu()
    new_imu.header.stamp = imu_msg.header.stamp  # preserve measurement time

    frame_override = rospy.get_param("~frame_id", "")
    new_imu.header.frame_id = frame_override if frame_override else imu_msg.header.frame_id

    new_imu.orientation = imu_msg.orientation
    new_imu.orientation_covariance = imu_msg.orientation_covariance
    new_imu.angular_velocity = imu_msg.angular_velocity
    new_imu.angular_velocity_covariance = imu_msg.angular_velocity_covariance

<<<<<<< HEAD
    # Overwrite linear acceleration with free accel
    new_imu.linear_acceleration.x = 0.0 if abs(free_acc_msg.vector.x) < THRESHOLD else free_acc_msg.vector.x
    new_imu.linear_acceleration.y = 0.0 if abs(free_acc_msg.vector.y) < THRESHOLD else free_acc_msg.vector.y
    new_imu.linear_acceleration.z = 0.0 if abs(free_acc_msg.vector.z) < THRESHOLD else free_acc_msg.vector.z
=======
    new_imu.linear_acceleration = free_acc_msg.vector
>>>>>>> bb8abd67 (added debug statements in superimposer, and made the following changes to the republisher files. In depth republisher, we assign big cov to everything but z -> avoid confclits, also the type is float which has no head. for dvl cov we capture a stamp once per line and reuse it, we used to have seperate call())
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
