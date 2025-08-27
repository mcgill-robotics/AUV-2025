import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

from tf.transformations import euler_from_quaternion

def pose_callback(msg: PoseStamped):
    # Camera position in map frame
    tx = msg.pose.position.x
    ty = msg.pose.position.y
    tz = msg.pose.position.z

    # Orientation quaternion
    q = msg.pose.orientation
    quat = [q.x, q.y, q.z, q.w]

    # Roll, pitch, yaw
    roll, pitch, yaw = euler_from_quaternion(quat)

    RAD2DEG = 57.295779513

    rospy.loginfo(
        "Received pose in '%s' frame : X: %.2f Y: %.2f Z: %.2f - "
        "R: %.2f P: %.2f Y: %.2f",
        msg.header.frame_id,
        tx, ty, tz,
        roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG,
    )

def depth_callback(msg: Image):
    # image center pixel indices
    u = msg.width  // 2
    v = msg.height // 2

    # linear index into the flattened array
    center_idx = u + msg.width * v

    # log the depth at the center pixel
    rospy.loginfo("Center distance : %f m", center_idx)


def main():
    rospy.init_node('zed_tracking_subscriber', anonymous=False)

    rospy.Subscriber(
        '/zed/zed_node/pose',
        PoseStamped,
        pose_callback,
        queue_size=10
    )

    rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, depth_callback);

    rospy.spin()

if __name__ == '__main__':
    main()
