#!/usr/bin/env python3

# Used for converting rosbag to png

import rospy
from sensor_msgs.msg import CompressedImage, Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import os

def image_callback_compressed(msg):
    global i
    bridge = CvBridge()
    np_img = bridge.compressed_imgmsg_to_cv2(msg)
    np_img.astype(np.float32)
    cv2.imwrite(pwd + '/images/bag_{}.png'.format(i), np_img)
    i += 1
    
def image_callback(msg):
    global i
    bridge = CvBridge()
    np_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imwrite(pwd + '/images/bag_{}.png'.format(i), np_img)
    i += 1
    
    
if __name__ == '__main__':
    i = 1
    pwd = os.path.realpath(os.path.dirname(__file__))
    rospy.init_node('bag_2_png')
    # rospy.Subscriber('/vision/front_cam/image_rgb', CompressedImage, image_callback_compressed)
    rospy.Subscriber('/vision/front_cam/image_rgb', Image, image_callback)
    rospy.spin()
