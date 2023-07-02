#!/usr/bin/env python3

import rospy

from tf2_ros import TransformBroadcaster 
from geometry_msgs.msg import TransformStamped, Pose

'''
Broadcasts the current coordinate transformation of the auv with respect to the world frame.
Listens to the pose topic to get the transformation, then broadcast it as a tf2 transformation.
The world frame is north east up.
'''

def auv_cb(pose):
    br = TransformBroadcaster()

    t = TransformStamped()
    
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"

    t.child_frame_id = "auv_base"

    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z 
    t.transform.rotation = pose.orientation

    t2 = TransformStamped()
    
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "world_rotation"

    t2.child_frame_id = "auv_rotation"

    t2.transform.translation.x = 0
    t2.transform.translation.y = 0
    t2.transform.translation.z = 0
    t2.transform.rotation = pose.orientation

    br.sendTransform(t)
    br.sendTransform(t2)


def world_cb(pose):
    br = TransformBroadcaster()

    t = TransformStamped()
    
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "global"

    t.child_frame_id = "world"

    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z 
    t.transform.rotation = pose.orientation

    br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('transform_broadcaster')
    rospy.Subscriber('pose', Pose, auv_cb, queue_size=50)
    rospy.Subscriber('pose_world', Pose, world_cb, queue_size=50)

    rospy.spin()
