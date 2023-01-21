#!/usr/bin/env python3

import numpy as np
import rospy

from sensor_msgs.msg import Image
from auv_msgs.msg import ObjectDetectionFrame

def detect_on_image(raw_img):
    model_filename = find_model()
    model = YOLO(model_filename)
    img = cv2(raw_img)
    detections = model(img)
    detectionFrame = ObjectDetectionFrame()
    detectionFrame.objects = []
    detectionFrame.camera = camera_name
    pub.publish(detectionFrame)

if __name__ == '__main__':
    rospy.init_node('object_detection')
    pub = rospy.Publisher('/viewframe_detection', ObjectDetectionFrame, queue_size=5)
    camera_name = "down_camera"
    sub = rospy.Subscriber('usb_cam/image_raw', Image, detect_on_image)
    rospy.spin()
