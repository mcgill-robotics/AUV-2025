#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Imu

class IMUCovarianceRepublisher:
    def __init__(self):
        self.raw_topic = rospy.get_param("~in_topic", "/sensors/imu/data")
        self.out_topic = rospy.get_param("~out_topic", "/sensors/imu/data_cov")
        self.window_size = rospy.get_param("~window_size", 200)    # samples
        self.compute_interval = rospy.get_param("~compute_interval", 1.0) 
        self.orientation_var = rospy.get_param("~orientation_variance", 0.01)  

        self.ang_buf = []
        self.acc_buf = []

        self.cov_ang = np.eye(3) * 1e-3
        self.cov_acc = np.eye(3) * 1e-2

        self.pub = rospy.Publisher(self.out_topic, Imu, queue_size=10)
        self.sub = rospy.Subscriber(self.raw_topic, Imu, self.raw_cb)

        rospy.Timer(rospy.Duration(self.compute_interval), self.compute_cov)

        rospy.loginfo(
            "IMU CovRepublisher:\n"
            "  in: %s\n  out: %s\n"
            "  window: %d samples\n  interval: %.2f s\n"
            "  orientation variance: %.5f",
            self.raw_topic, self.out_topic,
            self.window_size, self.compute_interval,
            self.orientation_var
        )

    def raw_cb(self, msg: Imu):
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration

        oc = [0.0]*9
        oc[0] = oc[4] = oc[8] = self.orientation_var
        out.orientation_covariance = oc

        out.angular_velocity_covariance = self.cov_ang.flatten().tolist()
        out.linear_acceleration_covariance = self.cov_acc.flatten().tolist()

        self.pub.publish(out)

        self.ang_buf.append([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
        self.acc_buf.append([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])

        if len(self.ang_buf) > self.window_size:
            self.ang_buf.pop(0)
            self.acc_buf.pop(0)

    def compute_cov(self, event):
        if len(self.ang_buf) < 2:
            return 
        av = np.array(self.ang_buf).T 
        la = np.array(self.acc_buf).T  
        self.cov_ang = np.cov(av)    
        self.cov_acc = np.cov(la)     
        rospy.logdebug(
            "Updated IMU covariances:\n"
            "  ang_vel:\n%s\n  accel:\n%s",
            self.cov_ang, self.cov_acc
        )

if __name__ == "__main__":
    rospy.init_node("imu_covariance_republisher")
    IMUCovarianceRepublisher()
    rospy.spin()
