#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
import numpy as np

def seperateRGBD(msg):
    #seperate into RGB and depth and re-publish

if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('parse_front_cam')

    rgb_pub = rospy.Publisher("/vision/front_cam/image_rgb", Image, queue_size=1)
    depth_pub = rospy.Publisher("/vision/front_cam/depth", Image, queue_size=1)
    raw_sub = rospy.Subscriber('/vision/front_cam/image_raw', Image, seperateRGBD)
    rospy.spin()
        
