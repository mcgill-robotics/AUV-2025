#!/usr/bin/env python3

import cv2
import rospy
import lane_marker_measure
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

#callback when an image is received
def threshold_image(raw_img):
    #only predict if i has not reached detect_every yet
    global i
    i += 1
    if i <= threshold_every: return
    i = 0
    #convert image to cv2
    img = bridge.imgmsg_to_cv2(raw_img, "bgr8")
    lane_marker_measure.thresholdRed(img, downscale_pub, blur1_pub, tol_pub, blur2_pub, thresh_pub)
    

if __name__ == '__main__':
    i = 0
    threshold_every = 1  #run the model every _ frames received (to not eat up too much RAM)
    #bridge is used to convert sensor_msg images to cv2
    bridge = CvBridge()
    rospy.init_node('debug_thresholding')
    sub = rospy.Subscriber('vision/down_cam/cropped', Image, threshold_image)
    downscale_pub = rospy.Publisher('vision/debug/lane_marker_downscale', Image, queue_size=1)
    blur1_pub = rospy.Publisher('vision/debug/lane_marker_blur1', Image, queue_size=1)
    tol_pub = rospy.Publisher('vision/debug/lane_marker_tolerance', Image, queue_size=1)
    blur2_pub = rospy.Publisher('vision/debug/lane_marker_blur2', Image, queue_size=1)
    thresh_pub = rospy.Publisher('vision/debug/lane_marker_threshold', Image, queue_size=1)
    
    rospy.spin()
