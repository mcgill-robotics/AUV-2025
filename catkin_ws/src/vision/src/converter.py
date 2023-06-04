#!/usr/bin/env python3

# Used for converting rosbag to png

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

def image_callback(data):
    global i
    bridge = CvBridge()
    np_img = bridge.compressed_imgmsg_to_cv2(data)
    np_img.astype(np.float32)
    cv2.imwrite('/home/felps/AUV-2023/catkin_ws/images/image{}.png'.format(i), np_img)
    i += 1
    
    
if __name__ == '__main__':
    i = 1
    rospy.init_node('bag_to_png_converter')
    rospy.Subscriber('/provider_vision/Front_GigE/compressed', CompressedImage, image_callback)
    rospy.spin()