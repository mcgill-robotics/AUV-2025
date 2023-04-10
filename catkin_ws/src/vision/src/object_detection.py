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
from auv_msgs.msg import ObjectMap
import math
import torch
class State:
    def __init__(self):
        self.x_pos_sub = rospy.Subscriber('state_x', Float64, self.updateX)
        self.y_pos_sub = rospy.Subscriber('state_y', Float64, self.updateY)
        self.z_pos_sub = rospy.Subscriber('state_z', Float64, self.updateZ)
        self.theta_x_sub = rospy.Subscriber('state_theta_x', Float64, self.updateThetaX)
        self.theta_y_sub = rospy.Subscriber('state_theta_y', Float64, self.updateThetaY)
        self.theta_z_sub = rospy.Subscriber('state_theta_z', Float64, self.updateThetaZ)
        self.x = None
        self.y = None
        self.z = None
        self.theta_x = None
        self.theta_y = None
        self.theta_z = None
        #add depth map here
def updateX(self, msg):
    self.x = float(msg.data)
def updateY(self, msg):
    self.y = float(msg.data)
def updateZ(self, msg):
    self.z = float(msg.data)
def updateThetaX(self, msg):
    self.theta_x = float(msg.data)
def updateThetaY(self, msg):
    self.theta_y = float(msg.data)
def updateThetaZ(self, msg):
    self.theta_z = float(msg.data)

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

#callback when an image is received
#runs model on image, publishes detection frame and generates/publishes visualization of predictions
def detect_on_image(raw_img, camera_id):
    current_state = (state.x, state.y, state.z, state.theta_x, state.theta_y, state.theta_z)
    if None in current_state: return
    #only predict if i has not reached detect_every yet
    global i
    i[camera_id] += 1
    if i[camera_id] <= detect_every: return
    i[camera_id] = 0
    #convert image to cv2
    img = bridge.imgmsg_to_cv2(raw_img, "bgr8")
    
    #run model on img
    detections = model[camera_id].predict(img, device=torch.device('cuda')) #change device for cuda
    #initialize empty arrays for object detection frame message
    label = []
    detection_confidence = []
    obj_x = []
    obj_y = []
    obj_z = []
    obj_theta_z = []
    pose_confidence = []
    extra_field = []
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
            detection_confidence.append(conf) 
            label.append(cls_id)
            #if a lane marker is detected on down cam then add heading visualization to image
            if camera_id == 0:
                if cls_id == 0:
                    headings, center, img = measureLaneMarker(img, bbox)
                    if None in headings:
                        extra_field.append(None)
                        obj_theta_z.append(None)
                        center = bbox
                    else:
                        obj_theta_z.append(state.theta_z + (headings[0]-90))
                        extra_field.append(state.theta_z + (headings[1]-90))
                #ADD ELIFs HERE TO MEASURE THE ROTATION OF OTHER CLASSES OF OBJECTS
                else:
                    extra_field.append(None)
                    obj_theta_z.append(None)
                    center = bbox
                #POSITION FOR OBJECTS ON DOWN CAM
                #ASSUME THEY ARE ON THE BOTTOM OF THE POOL, USE FOV TO FIND APPROXIMATE LOCATION
                #check that all of the down camera viewframe faces the pool bottom
                down_cam_vfov = down_cam_hfov*(h/w)
                if abs(state.theta_x) + down_cam_hfov/2 >= 90 or abs(state.theta_y) + down_cam_vfov/2 >= 90: 
                    obj_x.append(None)
                    obj_y.append(None)
                    obj_z.append(None)
                    pose_confidence.append(None)
                else:
                    #assuming FOV increases linearly with distance from center pixel
                    x_center_offset = (center[0]-(w/2)) / w #-0.5 to 0.5
                    x_angle_offset = state.theta_x + down_cam_hfov*x_center_offset
                    x_slope_offset = math.tan((x_angle_offset/180)*math.pi)
                    global_center_x = state.x + abs(-pool_depth - state.z)*x_slope_offset
                    y_center_offset = ((h/2)-center[1]) / h #negated since y goes from top to bottom
                    y_angle_offset = state.theta_y + down_cam_vfov*y_center_offset
                    y_slope_offset = math.tan((y_angle_offset/180)*math.pi)
                    global_center_y = state.y + abs(-pool_depth - state.z)*y_slope_offset

                    #confidence model:
                        # 0.25 at corners
                        # 0.5 at edges 
                        # 1.0 at center
                    x_conf = 1.0 - abs(x_center_offset)
                    y_conf = 1.0 - abs(y_center_offset)
                    pose_confidence.append(x_conf*y_conf) 

                    obj_x.append(global_center_x)
                    obj_y.append(global_center_y)
                    obj_z.append(-pool_depth) # assume the object is on the bottom of the pool
            else:
                #measure with stereo depth to get x,y,z
                #use custom object measurements to get theta_z
                obj_x.append(None)
                obj_y.append(None)
                obj_z.append(None)
                obj_theta_z.append(None)
                extra_field.append(None)
                pose_confidence.append(None)  #make inversely proportional to distance from object
            #add bbox visualization to img
            img = visualizeBbox(img, bbox, class_names[camera_id][cls_id] + " " + str(conf*100) + "%")
    #create object detection frame message and publish it
    detectionFrame = ObjectDetectionFrame()
    detectionFrame.label = label
    detectionFrame.detection_confidence = detection_confidence
    detectionFrame.obj_x = obj_x
    detectionFrame.obj_y = obj_y
    detectionFrame.obj_z = obj_z
    detectionFrame.obj_theta_z = obj_theta_z
    detectionFrame.pose_confidence = pose_confidence
    detectionFrame.extra_field = extra_field
    pub.publish(detectionFrame)
    #convert visualization image to sensor_msg image and publish it to corresponding cameras visualization topic
    img = bridge.cv2_to_imgmsg(img, "bgr8")
    visualisation_pubs[camera_id].publish(img)

BOX_COLOR = (255, 255, 255) # White
HEADING_COLOR = (255, 0, 0) # Blue
TEXT_COLOR = (0, 0, 0) # Black

detect_every = 10  #run the model every _ frames received (to not eat up too much RAM)
min_prediction_confidence = 0.6 #only report predictions with confidence at least 60%

pool_depth = 4
down_cam_hfov = 75

if __name__ == '__main__':
    state = State()

    #bridge is used to convert sensor_msg images to cv2
    bridge = CvBridge()
    #get and start models
    pwd = os.path.realpath(os.path.dirname(__file__))
    #init nodes and publishers/subscribers for each camera
    rospy.init_node('object_detection')
    
    #CHANGE FOR NEW CAMERAS/MODELS:
    down_cam_model_filename = pwd + "/models/down_cam_model.pt"
    model = [
        YOLO(down_cam_model_filename)
        ]
    for m in model: m.to(torch.device('cuda'))
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