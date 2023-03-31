#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
    
if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('stream_cameras')

    sensor_id = rospy.get_param('~sensor_id')
    inputImgWidth = rospy.get_param('~inputImgWidth')
    inputImgHeight = rospy.get_param('~inputImgHeight')
    outputImgWidth = rospy.get_param('~outputImgWidth')
    outputImgHeight = rospy.get_param('~outputImgHeight')
    framerate = rospy.get_param('~framerate')
    outputTopic = rospy.get_param('~outputTopic')
    
    pipeline = "nvarguscamerasrc sensor-id={} ! video/x-raw(memory:NVMM),width={},height={},framerate={}/1 ! nvvidconv ! video/x-raw(memory:NVMM),width={},height={},framerate={}/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink".format(sensor_id, inputImgWidth, inputImgHeight, framerate, outputImgWidth, outputImgHeight, framerate)
    
    pub = rospy.Publisher(outputTopic, Image, queue_size=1)
    
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Failed to open camera")
        exit()
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            print("Error reading frame from camera")
            break
        pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    cap.release()
        
