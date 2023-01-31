#!/usr/bin/env python3

import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
import os
from ultralytics import YOLO

from sensor_msgs.msg import Image
from auv_msgs.msg import ObjectDetectionFrame

def detect_on_image(raw_img, camera_id):
    img = bridge.imgmsg_to_cv2(raw_img, "bgr8")
    detections = model(img)
    label = []
    bounding_box_x = []
    bounding_box_y = []
    bounding_box_width = []
    bounding_box_height = []
    confidence = []
    for detection in detections:
        box = detection.boxes
        print(box.xywh)
        print(box.cls)
        print(box.conf)
        xywh = box.xywh.numpy()
        bounding_box_x.append(xywh[0])
        bounding_box_y.append(xywh[1])
        bounding_box_width.append(xywh[2])
        bounding_box_height.append(xywh[3])
        confidence.append(box.conf.numpy()[0]) 
        label.append(int(box.cls.numpy()[0]))

    detectionFrame = ObjectDetectionFrame()
    detectionFrame.label = label
    detectionFrame.bounding_box_x = bounding_box_x
    detectionFrame.bounding_box_y = bounding_box_y
    detectionFrame.bounding_box_width = bounding_box_width
    detectionFrame.bounding_box_height = bounding_box_height
    detectionFrame.confidence = confidence
    detectionFrame.camera = camera_id
    
    pub.publish(detectionFrame)

if __name__ == '__main__':
    bridge = CvBridge()
    pwd = os.path.realpath(os.path.dirname(__file__))
    model_filename = pwd + "/last.pt"
    model = YOLO(model_filename)
    rospy.init_node('object_detection')
    pub = rospy.Publisher('/viewframe_detection', ObjectDetectionFrame, queue_size=5)
    #copy paste subscriber for additional cameras (change integer so there is a unique int for each camera)
    sub = rospy.Subscriber('usb_cam/image_raw', Image, detect_on_image, 0)
    rospy.spin()
