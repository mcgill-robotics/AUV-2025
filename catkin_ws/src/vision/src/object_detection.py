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
        boxes = detection.boxes.numpy()
        for box in boxes:
            if float(list(box.conf)[0]) < min_prediction_confidence:
                continue
            bbox = list(box.xywh[0])
            h, w, channels = img.shape
            xywh = box.xywh.numpy()
            bounding_box_x.append(bbox[0]/w)
            bounding_box_y.append(bbox[1]/h)
            bounding_box_width.append(bbox[2]/w)
            bounding_box_height.append(bbox[3]/h)
            confidence.append(box.conf.numpy()[0]) 
            label.append(int(list(box.cls)[0]))
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
    min_prediction_confidence = 0.5
    bridge = CvBridge()
    pwd = os.path.realpath(os.path.dirname(__file__))
    model_filename = pwd + "/last.pt"
    model = YOLO(model_filename)
    rospy.init_node('object_detection')
    pub = rospy.Publisher('/viewframe_detection', ObjectDetectionFrame, queue_size=5)
    #copy paste subscriber for additional cameras (change integer so there is a unique int for each camera)
    sub = rospy.Subscriber('usb_cam/image_raw', Image, detect_on_image, 0)
    rospy.spin()
