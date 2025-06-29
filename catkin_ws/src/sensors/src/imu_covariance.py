#!/usr/bin/env python3

"""
IMU Covariance Republisher Node

This ROS node subscribes to raw IMU data (sensor_msgs/Imu) from the configured topic,
and republishes it with updated covariance matrices for orientation, angular velocity,
and linear acceleration.

- Uses manufacturer specifications to calculate conservative default variances for
  angular velocity and linear acceleration.
- Uses the IMU-provided orientation covariance if available; otherwise falls back
  to a configured default variance.
- Publishes the republished IMU data with covariances to a new topic for downstream EKF.

Datasheet reference:
https://support.sbg-systems.com/sc/el/latest/ellipse-documentation/performance-specifications/raw-imu-specifications 
"""

import rospy
import numpy as np
from sensor_msgs.msg import Imu

class IMUCovarianceRepublisher:
    def __init__(self):
        # Load ROS parameters for topics and variances
        self.raw_topic = rospy.get_param("~in_topic", "/sensors/imu/data")
        self.out_topic = rospy.get_param("~out_topic", "/sensors/imu/data_cov")
        self.orientation_var = rospy.get_param("~orientation_variance", 0.01)  # fallback if no dynamic orientation covariance available

        # Load manufacturer-based IMU noise specs, with safety factor applied
        self.bandwidth_acc = rospy.get_param("~acc_bandwidth", 390)  # Hz, effective accelerometer bandwidth
        self.bandwidth_gyro = rospy.get_param("~gyro_bandwidth", 133)  # Hz, effective gyro bandwidth
        self.safety_factor = rospy.get_param("~safety_factor", 10)  # Multiplier for conservative variance estimation

        # Calculate linear acceleration variance:
        # (noise density in μg → m/s²)^2 * bandwidth * safety factor
        self.linear_acc_var = (57e-6 * 9.81)**2 * self.bandwidth_acc * self.safety_factor

        # Calculate angular velocity variance:
        # ((ARW / 60) to get deg/s/√Hz → rad/s/√Hz)^2 * bandwidth * safety factor
        self.ang_vel_var = ((0.18/60) * (np.pi/180))**2 * self.bandwidth_gyro * self.safety_factor

        # Initialize covariance matrices with computed variances
        self.cov_ang = np.eye(3) * self.ang_vel_var  # angular velocity covariance (rad/s)^2
        self.cov_acc = np.eye(3) * self.linear_acc_var  # linear acceleration covariance (m/s^2)^2

        # ROS publisher and subscriber
        self.pub = rospy.Publisher(self.out_topic, Imu, queue_size=10)
        self.sub = rospy.Subscriber(self.raw_topic, Imu, self.raw_cb)

        # Log configuration summary
        rospy.loginfo(
            "IMU CovRepublisher initialized.\n"
            "  Input topic: %s\n"
            "  Output topic: %s\n"
            "  Orientation variance fallback: %.5f",
            self.raw_topic, self.out_topic, self.orientation_var
        )

    def raw_cb(self, msg: Imu):
        # Create output message copying raw IMU data
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration

        # Orientation covariance: use provided if valid, else fallback
        if msg.orientation_covariance[0] >= 0:
            out.orientation_covariance = msg.orientation_covariance
        else:
            oc = [0.0]*9
            oc[0] = oc[4] = oc[8] = self.orientation_var
            out.orientation_covariance = oc

        # Set fixed angular velocity and linear acceleration covariance based on manufacturer specs
        out.angular_velocity_covariance = self.cov_ang.flatten().tolist()
        out.linear_acceleration_covariance = self.cov_acc.flatten().tolist()

        # Publish updated IMU message
        self.pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("imu_covariance_republisher")
    IMUCovarianceRepublisher()
    rospy.spin()
