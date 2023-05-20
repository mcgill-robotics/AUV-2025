#!/usr/bin/env python3

import rospy

from tf2_ros import TransformBroadcaster 
from geometry_msgs.msg import TransformStamped, Pose

'''
Broadcasts the current coordinate transformation of the auv with respect to the world frame.
Listens to the pose topic to get the transformation, then broadcast it as a tf2 transformation.
The world frame is north east up.
'''

def state_cb(pose):
    br = TransformBroadcaster()

    t = TransformStamped()
    
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"

    t.child_frame_id = "auv_base"

    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z 
    t.transform.rotation = pose.orientation

    br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('transform_broadcaster')
    rospy.Subscriber('pose', Pose, state_cb, queue_size=50)
    rospy.spin()
