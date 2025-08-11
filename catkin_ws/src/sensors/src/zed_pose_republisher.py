#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_matrix, quaternion_from_matrix

''' https://www.stereolabs.com/docs/ros '''

def Rt(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def quat_to_R(q):  # geometry_msgs/Quaternion -> 3x3 rotation matrix
    return quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

class ZedToAuvCompose:
    """
    Subscribes: PoseWithCovarianceStamped (pose of zed in 'map')
    Looks up:   static TF zed_frame -> auv_frame
    Publishes:  PoseWithCovarianceStamped (pose of auv in 'map')
    Covariance: passed through unchanged (no rotation)
    """
    def __init__(self):
        self.in_topic  = rospy.get_param("~input_topic",  "/zed/zed_node/pose_with_covariance") #TODO: verify the topic name. 
        self.out_topic = rospy.get_param("~output_topic", "sensors/zed/pose")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.zed_frame = rospy.get_param("~zed_frame", "zed")
        self.auv_frame = rospy.get_param("~auv_frame", "auv")

        self.buf = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.listener = tf2_ros.TransformListener(self.buf)
        self.pub = rospy.Publisher(self.out_topic, PoseWithCovarianceStamped, queue_size=10)
        rospy.Subscriber(self.in_topic, PoseWithCovarianceStamped, self.cb, queue_size=10)

        rospy.loginfo("zed_pose_to_auv_pose_tf2_compose: input=%s output=%s map=%s zed=%s auv=%s",
                      self.in_topic, self.out_topic, self.map_frame, self.zed_frame, self.auv_frame)

    def cb(self, msg: PoseWithCovarianceStamped):
        # 1) Build T_map_zed from the incoming message (POSE OF ZED IN MAP)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        T_map_zed = Rt(quat_to_R(q), np.array([p.x, p.y, p.z], dtype=float))

        # 2) Get the fixed T_zed_auv from TF (you should publish auv->zed as static; TF will invert as needed)
        try:
            tf = self.buf.lookup_transform(self.zed_frame, self.auv_frame,
                                           msg.header.stamp, rospy.Duration(0.1)).transform
        except Exception as e:
            rospy.logwarn_throttle(1.0, "Waiting for static %s -> %s: %s",
                                   self.zed_frame, self.auv_frame, str(e))
            return

        R_za = quaternion_matrix([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])[:3, :3]
        t_za = np.array([tf.translation.x, tf.translation.y, tf.translation.z], dtype=float)
        T_zed_auv = Rt(R_za, t_za)

        # 3) Compose: T_map_auv = T_map_zed * T_zed_auv
        T_map_auv = T_map_zed.dot(T_zed_auv)
        R_out, t_out = T_map_auv[:3, :3], T_map_auv[:3, 3]
        q_out = quaternion_from_matrix(Rt(R_out, [0, 0, 0]))

        # 4) Publish AUV pose in map (covariance passed through unchanged)
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame
        out.pose.pose.position.x = float(t_out[0])
        out.pose.pose.position.y = float(t_out[1])
        out.pose.pose.position.z = float(t_out[2])
        out.pose.pose.orientation = Quaternion(x=float(q_out[0]), y=float(q_out[1]),
                                               z=float(q_out[2]), w=float(q_out[3]))
        out.pose.covariance = msg.pose.covariance  # pass-through
        self.pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("zed_pose_republisher")
    ZedToAuvCompose()
    rospy.spin()
