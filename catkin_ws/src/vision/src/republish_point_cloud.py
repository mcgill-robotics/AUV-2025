#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np


import image_geometry

if __name__ == "__main__":
    rospy.init_node("republish_point_cloud")
    pub = rospy.Publisher("/rviz/point_cloud_view", PointCloud2, queue_size=1)
    def callback(msg):
        time = rospy.Time(0)
        pc = point_cloud2.read_points_list(msg)
        pc = np.array(pc)
        pc = pc[:,[2,0,1]]
        pc[:,2] *= -1
        new_msg = point_cloud2.create_cloud_xyz32(msg.header, pc)
        new_msg.header.stamp = time
        new_msg.header.frame_id = "auv_base"
        pub.publish(new_msg)
    rospy.Subscriber("/vision/front_cam/point_cloud", PointCloud2, callback, queue_size=1)
    rospy.spin()