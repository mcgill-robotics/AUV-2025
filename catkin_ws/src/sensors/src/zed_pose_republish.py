#!/usr/bin/env python3
import rospy, numpy as np, tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_matrix, quaternion_from_matrix

def Rt(R, t):
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = t; return T

def quat_to_R(q):
    return quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

def rotate_pose_cov(cov_list, R):
    C = np.array(cov_list, dtype=float).reshape(6,6)
    J = np.zeros((6,6)); J[:3,:3] = R; J[3:,3:] = R
    return (J @ C @ J.T).reshape(-1).tolist() #we didnt roate covariances into auv frame

class ZedToAuvCompose:
    def __init__(self):
        self.in_topic  = rospy.get_param("~input_topic",  "/zed2i/zed_node/pose_with_covariance")
        self.out_topic = rospy.get_param("~output_topic", "sensors/zed2i/pose")
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
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        T_map_zed = Rt(quat_to_R(q), np.array([p.x, p.y, p.z], dtype=float))

        # ensure TF is available at the measurement time
        if not self.buf.can_transform(self.zed_frame, self.auv_frame, msg.header.stamp, rospy.Duration(0.2)):
            rospy.logwarn_throttle(1.0, "No TF %s->%s at t=%.3f", self.zed_frame, self.auv_frame, msg.header.stamp.to_sec())
            return

        tf = self.buf.lookup_transform(self.zed_frame, self.auv_frame, msg.header.stamp, rospy.Duration(0.2)).transform
        R_za = quaternion_matrix([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])[:3, :3]
        t_za = np.array([tf.translation.x, tf.translation.y, tf.translation.z], dtype=float)
        T_zed_auv = Rt(R_za, t_za)

        T_map_auv = T_map_zed.dot(T_zed_auv)
        R_out, t_out = T_map_auv[:3,:3], T_map_auv[:3,3]
        q_out = quaternion_from_matrix(Rt(R_out, [0,0,0]))

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, t_out)
        out.pose.pose.orientation = Quaternion(x=float(q_out[0]), y=float(q_out[1]), z=float(q_out[2]), w=float(q_out[3]))
        out.pose.covariance = rotate_pose_cov(msg.pose.covariance, R_za)  # <<< rotate into AUV frame
        self.pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("zed_pose_republisher")
    ZedToAuvCompose()
    rospy.spin()