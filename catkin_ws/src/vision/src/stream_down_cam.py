#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('stream_cameras')

    #get parameters from launch file
    sensor_id = rospy.get_param('~sensor_id')
    outputImgWidth = rospy.get_param('~outputImgWidth')
    outputImgHeight = rospy.get_param('~outputImgHeight')
    framerate = rospy.get_param('~framerate')
    outputTopic = rospy.get_param('~outputTopic')
    distortion_model = 'plumb_bob'
    
    #define gstreamer pipeline to stream camera feeds
    pipeline = "nvarguscamerasrc sensor-id={} ! video/x-raw(memory:NVMM),width=1920,height=1080,format=(string)NV12,framerate=30/1 ! nvvidconv ! video/x-raw(memory:NVMM), width={}, height={}, format=(string)NV12, framerate={}/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink".format(sensor_id, outputImgWidth, outputImgHeight, framerate)

    #define output topics for the camera images
    pub = rospy.Publisher("/vision" + outputTopic + '/image_raw', Image, queue_size=1)
    
    #START GSTREAM using defined pipeline
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Failed to open camera")
        exit()
    #continuously read frames from the camera feed
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            print("Error reading frame from camera")
            break
        msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub.publish(msg)
    cap.release()
        
