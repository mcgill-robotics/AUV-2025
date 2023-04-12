#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
import numpy as np

def disparity_to_depth(msg, _):
    focal_length = float(msg.f)
    baseline = float(msg.T)
    disparity_msg = msg.image
    disparity_img = bridge.imgmsg_to_cv2(disparity_msg, "passthrough")
    depth_img = (focal_length*baseline) / disparity_img
    depth_msg = bridge.cv2_to_imgmsg(depth_img, "passthrough")
    depth_pub.publish(depth_msg)

if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('depth_map')

    depth_pub = rospy.Publisher("/vision/stereo/depth_map", Image, queue_size=1)
    disparity_sub = rospy.Subscriber('/vision/stereo/disparity', DisparityImage, disparity_to_depth, 0)
    rospy.spin()
        
