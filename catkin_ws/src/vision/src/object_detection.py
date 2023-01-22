#!/usr/bin/env python3

import numpy as np
import rospy
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from auv_msgs.msg import ObjectDetectionFrame

def detect_on_image(raw_img):
    img = bridge.imgmsg_to_cv2(raw_img, "bgr8")
    detections = model(img)
    for results in detections:
        print()
    detections = (detections.boxes.boxes.numpy())
    classes = []
    bounding_box_x
    bounding_box_y
    bounding_box_width
    bounding_box_height
    for detection in detections:
    print(detections)
    detectionFrame = ObjectDetectionFrame()
    detectionFrame.objects = []
    detectionFrame.camera = camera_id
    pub.publish(detectionFrame)

if __name__ == '__main__':
    bridge = CvBridge()
    pwd = os.path.realpath(os.path.dirname(__file__))
    model_filename = pwd + "/model.pt"
    model = YOLO(model_filename)
    rospy.init_node('object_detection')
    pub = rospy.Publisher('/viewframe_detection', ObjectDetectionFrame, queue_size=5)
    camera_id = 0
    sub = rospy.Subscriber('usb_cam/image_raw', Image, detect_on_image)
    rospy.spin()
