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
HEADING_COLOR = (255, 0, 0) # Blue
TEXT_COLOR = (0, 0, 0) # Black

#given an image, class name, and a bounding box, draws the bounding box rectangle and label name onto the image
def visualizeBbox(img, bbox, class_name, color=BOX_COLOR, thickness=2, fontSize=0.5):
    #get xmin, xmax, ymin, ymax from bbox 
    x_center, y_center, w, h = bbox
    x_min = x_center - w/2
    y_min = y_center - h/2
    x_min, x_max, y_min, y_max = int(x_min), int(x_min + w), int(y_min), int(y_min + h)
    #draw bounding box on image
    cv2.rectangle(img, (x_min, y_min), (x_max, y_max), color=color, thickness=thickness)
    #get size of class name text
    ((text_width, text_height), _) = cv2.getTextSize(class_name, cv2.FONT_HERSHEY_SIMPLEX, fontSize, 1)  
    #draw box around class name label on image
    cv2.rectangle(img, (x_min, y_min - int(1.3 * text_height)), (x_min + text_width, y_min), BOX_COLOR, -1)
    #put class name text in the box we drew
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

#given a bounding box and image, returns the image cropped to the bounding box (to isolate detected objects)
def cropToBbox(img, bbox):
    x_center, y_center, w, h = bbox
    x_min = x_center - w/2
    y_min = y_center - h/2
    x_min, x_max, y_min, y_max = int(x_min), int(x_min + w), int(y_min), int(y_min + h)
    crop_img = img[y_min:y_max, x_min:x_max]
    return crop_img

def measureLaneMarker(img, bbox):
    #crop image to lane marker
    cropped_img = cropToBbox(img, bbox)
    line_thickness = 2 # in pixels
    line_length = 0.25*min(bbox[2], bbox[3]) #line will be size of shortest bounding box side
    #measure headings from lane marker
    cropped_img_to_pub = bridge.cv2_to_imgmsg(cropped_img, "bgr8")
    cropped_img_pub.publish(cropped_img_to_pub)
    headings, center_point = lane_marker_measure.measure_headings(cropped_img)
    if None in (headings, center_point): return (None, None), (None, None), img
    center_point_x = center_point[0] + bbox[0] - bbox[2]/2
    center_point_y = center_point[1] + bbox[1] - bbox[3]/2
    center_point = (int(center_point_x), int(center_point_y))
    for angle in headings:
        #get angle, line start and line end from heading slope
        slope = math.tan((angle/-180)*math.pi)
        #calculate line x length from total length
            #line_length = sqrt(line_x_length^2 + line_y_length^2)
            #line_length^2 = line_x_length^2 + (line_x_length*slope)^2
            #line_length^2 = line_x_length^2 * (1 + slope^2)
            #line_x_length = sqrt(line_length^2 / (1 + slope^2))
        line_x_length = math.sqrt((line_length ** 2) / (1 + slope ** 2))
        if abs(angle) > 90: #heading goes into negative x
            line_end = (int(center_point[0]-line_x_length), int(center_point[1] - slope*line_x_length)) # (x,y)
        else: # heading goes into positive x
            line_end = (int(center_point[0]+line_x_length), int(center_point[1] + slope*line_x_length)) # (x,y)
        #draw line on original image
        cv2.line(img, center_point, line_end, HEADING_COLOR, line_thickness)
        #add text with measured angle of line at the end of the line
        cv2.putText(
            img,
            text=str(angle) + " deg.",
            org=line_end,
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.4, 
            color=HEADING_COLOR, 
            lineType=cv2.LINE_AA,
        )
    cv2.circle(img, center_point, radius=5, color=HEADING_COLOR, thickness=-1)
    return headings, center_point, img

#increase intensity of blues in given image
def make_bluer(img, color_shift_intensity):
    img_b, img_g, img_r = cv2.split(img) #split by channel
    img_b = np.uint16(img_b)
    img_b += color_shift_intensity
    np.clip(img_b, 0, 255, out=img_b)
    img_b = np.uint8(img_b)
    img = cv2.merge((img_b, img_g, img_r)) #merge adjusted channels
    return img

#callback when an image is received
#runs model on image, publishes detection frame and generates/publishes visualization of predictions
def detect_on_image(raw_img, camera_id):
    #only predict if i has not reached detect_every yet
    global i
    i[camera_id] += 1
    if i[camera_id] <= detect_every: return
    i[camera_id] = 0
    #convert image to cv2
    img = bridge.imgmsg_to_cv2(raw_img, "bgr8")
    
    #FOR TESTING
    img = make_bluer(img, color_shift_intensity=int(0.2*255))
    
    #run model on img
    detections = model[camera_id].predict(img, device=0) #change device for cuda
    #initialize empty arrays for object detection frame message
    label = []
    bounding_box_x = []
    bounding_box_y = []
    bounding_box_width = []
    bounding_box_height = []
    confidence = []
    heading1 = []
    heading2 = []
    center_x = []
    center_y = []
    #nested for loops get all predictions made by model
    for detection in detections:
        boxes = detection.boxes.cpu().numpy()
        for box in boxes:
            conf = float(list(box.conf)[0])
            #only consider predictinon if confidence is at least min_prediction_confidence
            if conf < min_prediction_confidence:
                continue
            #get bbox, class id
            bbox = list(box.xywh[0])
            cls_id = int(list(box.cls)[0])
            #add bbox and class id to viewframe arrays
            #scale bbox so it is a fraction of image size and not in pixels
            h, w, channels = img.shape
            xywh = box.xywh
            bounding_box_x.append(bbox[0]/w)
            bounding_box_y.append(bbox[1]/h)
            bounding_box_width.append(bbox[2]/w)
            bounding_box_height.append(bbox[3]/h)
            confidence.append(conf) 
            label.append(cls_id)
            #if a lane marker is detected on down cam then add heading visualization to image
            if cls_id == 0 and camera_id == 0:
                headings, center, img = measureLaneMarker(img, bbox)
                heading1.append(headings[0])
                heading2.append(headings[1])
                center_x.append(center[0])
                center_y.append(center[1])
            else:
                heading1.append(None)
                heading2.append(None)
                center_x.append(None)
                center_y.append(None)
            #add bbox visualization to img
            img = visualizeBbox(img, bbox, class_names[camera_id][cls_id] + " " + str(conf*100) + "%")
    #create object detection frame message and publish it
    detectionFrame = ObjectDetectionFrame()
    detectionFrame.label = label
    detectionFrame.bounding_box_x = bounding_box_x
    detectionFrame.bounding_box_y = bounding_box_y
    detectionFrame.bounding_box_width = bounding_box_width
    detectionFrame.bounding_box_height = bounding_box_height
    detectionFrame.confidence = confidence
    detectionFrame.camera = camera_id
    detectionFrame.heading1 = heading1
    detectionFrame.heading2 = heading2
    detectionFrame.center_x = center_x
    detectionFrame.center_y = center_y
    pub.publish(detectionFrame)
    #convert visualization image to sensor_msg image and publish it to corresponding cameras visualization topic
    img = bridge.cv2_to_imgmsg(img, "bgr8")
    visualisation_pubs[camera_id].publish(img)

if __name__ == '__main__':
    detect_every = 10  #run the model every _ frames received (to not eat up too much RAM)
    #only report predictions with confidence at least 40%
    min_prediction_confidence = 0.6
    #bridge is used to convert sensor_msg images to cv2
    bridge = CvBridge()
    #get and start models
    pwd = os.path.realpath(os.path.dirname(__file__))
    #init nodes and publishers/subscribers for each camera
    rospy.init_node('object_detection')
    
    #CHANGE FOR NEW CAMERAS/MODELS:
    down_cam_model_filename = pwd + "/models/model_v2.pt"
    model = [
        YOLO(down_cam_model_filename)
        ]
    #count for number of images received per camera
    i = [
        0
        ]
    class_names = [ #one array per camera, name index should be class id
        ["Lane Marker"]
        ]
    #one publisher per camera
    visualisation_pubs = [
        rospy.Publisher('vision/down_visual', Image, queue_size=1)
        ]
    #copy paste subscriber for additional cameras (change last argument so there is a unique int for each camera)
    #the int argument will be used to index debug publisher, model, class names, and i
    subs = [
        rospy.Subscriber('/vision/down_cam/image_raw', Image, detect_on_image, 0)
        ]
 	
    cropped_img_pub = rospy.Publisher('vision/debug/cropped', Image, queue_size=1)
    pub = rospy.Publisher('vision/viewframe_detection', ObjectDetectionFrame, queue_size=1)
    rospy.spin()
