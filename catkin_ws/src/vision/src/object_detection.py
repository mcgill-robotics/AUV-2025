#!/usr/bin/env python3

import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
import os
from ultralytics import YOLO
import lane_marker_measure
from sensor_msgs.msg import Image
from auv_msgs.msg import ObjectDetectionFrame
import math

BOX_COLOR = (255, 255, 255) # White
HEADING_COLOR = (0, 0, 255) # Red
TEXT_COLOR = (0, 0, 0) # Black

def visualize_bbox(img, bbox, class_name, color=BOX_COLOR, thickness=2, fontSize=0.5):
    x_center, y_center, w, h = bbox
    x_min = x_center - w/2
    y_min = y_center - h/2
    x_min, x_max, y_min, y_max = int(x_min), int(x_min + w), int(y_min), int(y_min + h)
    cv2.rectangle(img, (x_min, y_min), (x_max, y_max), color=color, thickness=thickness)
    ((text_width, text_height), _) = cv2.getTextSize(class_name, cv2.FONT_HERSHEY_SIMPLEX, fontSize, 1)    
    cv2.rectangle(img, (x_min, y_min - int(1.3 * text_height)), (x_min + text_width, y_min), BOX_COLOR, -1)
    cv2.putText(
        img,
        text=class_name,
        org=(x_min, y_min - int(0.3 * text_height)),
        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
        fontScale=fontSize, 
        color=TEXT_COLOR, 
        lineType=cv2.LINE_AA,
    )
    return img

def cropToBbox(img, bbox):
    x_center, y_center, w, h = bbox
    x_min = x_center - w/2
    y_min = y_center - h/2
    x_min, x_max, y_min, y_max = int(x_min), int(x_min + w), int(y_min), int(y_min + h)
    crop_img = img[y_min:y_max, x_min:x_max]
    return crop_img

def detect_on_image(raw_img, camera_id):
    global i
    i += 1
    if i % detect_every != 0: return
    i = 0
    img = bridge.imgmsg_to_cv2(raw_img, "bgr8")
    detections = model(img)
    label = []
    bounding_box_x = []
    bounding_box_y = []
    bounding_box_width = []
    bounding_box_height = []
    confidence = []
    for detection in detections:
        boxes = detection.boxes.cpu().numpy()
        for box in boxes:
            if float(list(box.conf)[0]) < min_prediction_confidence:
                continue
            bbox = list(box.xywh[0])
            h, w, channels = img.shape
            xywh = box.xywh
            bounding_box_x.append(bbox[0]/w)
            bounding_box_y.append(bbox[1]/h)
            bounding_box_width.append(bbox[2]/w)
            bounding_box_height.append(bbox[3]/h)
            confidence.append(box.conf[0]) 
            cls_id = int(list(box.cls)[0])
            label.append(cls_id)
            if cls_id == 0: #add lane marker heading lines to image
                cropped_img = cropToBbox(img, bbox)
                headings = lane_marker_measure.measure_headings(cropped_img)
                line_thickness = 1 # in pixels
                line_x_length = int(0.75*bbox[2]) #in pixels, will be 3/4 of bounding box width
                for slope in headings:
                    angle = math.degrees(math.atan(slope))
                    heading_start = (int(max(bbox[0]-line_x_length, 0)), int(max(bbox[1] - slope*line_x_length, 0))) # (x,y)
                    heading_end = (int(bbox[0]+line_x_length), int(bbox[1] + slope*line_x_length)) # (x,y)
                    cv2.line(img, heading_start, heading_end, HEADING_COLOR, line_thickness)
                    cv2.putText(
                        img,
                        text=str(-1*angle) + " deg.",
                        org=heading_end,
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.4, 
                        color=HEADING_COLOR, 
                        lineType=cv2.LINE_AA,
                    )
            img = visualize_bbox(img, bbox, class_names[cls_id] + " " + str(box.conf[0]*100) + "%")
    
    detectionFrame = ObjectDetectionFrame()
    detectionFrame.label = label
    detectionFrame.bounding_box_x = bounding_box_x
    detectionFrame.bounding_box_y = bounding_box_y
    detectionFrame.bounding_box_width = bounding_box_width
    detectionFrame.bounding_box_height = bounding_box_height
    detectionFrame.confidence = confidence
    detectionFrame.camera = camera_id
    pub.publish(detectionFrame)
    img = bridge.cv2_to_imgmsg(img, "bgr8")
    debug_pubs[camera_id].publish(img)

if __name__ == '__main__':
    detect_every = 10
    i = 0
    class_names = ["Lane Marker"] #index should be class id
    min_prediction_confidence = 0.4
    bridge = CvBridge()
    pwd = os.path.realpath(os.path.dirname(__file__))
    model_filename = pwd + "/last.pt"
    model = YOLO(model_filename)
    rospy.init_node('object_detection')
    pub = rospy.Publisher('viewframe_detection', ObjectDetectionFrame, queue_size=1)
    #one publisher per camera
    debug_pubs = [
        rospy.Publisher('downwards_cam_visual', Image, queue_size=1)
    ]
    #copy paste subscriber for additional cameras (change integer so there is a unique int for each camera)
    sub = rospy.Subscriber('usb_cam/image_raw', Image, detect_on_image, 0)
    rospy.spin()
