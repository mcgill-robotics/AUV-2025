#!/usr/bin/env python3

# Used for converting rosbag to png

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
import os

def image_callback(data):
    global i
    bridge = CvBridge()
    np_img = bridge.compressed_imgmsg_to_cv2(data)
    np_img.astype(np.float32)
    cv2.imwrite(pwd + '/images/bag_{}.png'.format(i), np_img)
    i += 1
    
    
if __name__ == '__main__':
    i = 1
    pwd = os.path.realpath(os.path.dirname(__file__))
    rospy.init_node('bag_2_png')
    rospy.Subscriber('/provider_vision/Front_GigE/compressed', CompressedImage, image_callback)
    rospy.spin()
