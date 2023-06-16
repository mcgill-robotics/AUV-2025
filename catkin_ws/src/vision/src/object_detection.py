#!/usr/bin/env python3

import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
import os
from ultralytics import YOLO
from sensor_msgs.msg import Image
from auv_msgs.msg import ObjectDetectionFrame
from object_detection_utils import *
import torch

#callback when an image is received
#runs model on image, publishes detection frame and generates/publishes visualization of predictions
def detect_on_image(raw_img, camera_id):
    #only predict if i has not reached detect_every yet
    global i
    i[camera_id] += 1
    if i[camera_id] <= detect_every: return
    i[camera_id] = 0
    
    current_states = {"x:", state.x, "y:", state.y, "z", state.z, "theta_x", state.theta_x, "theta_y", state.theta_y, "theta_z", state.theta_z}
    if None in current_states.values():
        print("State information missing. Skipping detection.")
        print(current_states)
        return
    #convert image to cv2
    img = bridge.imgmsg_to_cv2(raw_img, "bgr8")
    debug_img = np.copy(img)
    
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
            
            bbox = list(box.xywh[0])
            cls_id = int(list(box.cls)[0])
            global_class_id = global_class_ids[class_names[camera_id][cls_id]]
            label.append(global_class_id)
            detection_confidence.append(conf)
            #add bbox visualization to img
            debug_img = visualizeBbox(debug_img, bbox, class_names[camera_id][cls_id] + " " + str(conf*100) + "%")

            img_h, img_w, _ = img.shape
            if camera_id == 0: # DOWN CAM
                if global_class_id == 0: #LANE MARKER
                    pred_obj_x, pred_obj_y, pred_obj_z, pose_conf = getObjectPosition(center[0], center[1], img_h, img_w, z_pos=lane_marker_z)
                    pose_confidence.append(pose_conf) 
                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 

                    headings, center, debug_img = measureLaneMarker(img, bbox, debug_img)
                    if None in headings:
                        extra_field.append(None)
                        obj_theta_z.append(None)
                        center = bbox
                    else:
                        obj_theta_z.append(state.theta_z + (headings[0]-90))
                        extra_field.append(state.theta_z + (headings[1]-90))
                elif global_class_id == 4: # OCTAGON
                    pred_obj_x, pred_obj_y, pred_obj_z, pose_conf = getObjectPosition(bbox[0], bbox[1], img_h, img_w, z_pos=octagon_z)
                    pose_confidence.append(pose_conf) 
                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 
                    extra_field.append(None)
                    obj_theta_z.append(None)
            else: # FORWARD CAM
                depth_cropped = cropToBbox(state.depth, bbox)
                dist_from_camera = object_depth(depth_cropped, global_class_id)
                if global_class_id == 0: # LANE MARKER
                    pred_obj_x, pred_obj_y, pred_obj_z, pose_conf = getObjectPosition(center[0], center[1], img_h, img_w, dist_from_camera=dist_from_camera)
                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 
                    obj_theta_z.append(None)
                    extra_field.append(None)
                elif global_class_id == 1: # GATE
                    theta_z = measureGateAngle(depth_cropped)
                    pred_obj_x, pred_obj_y, pred_obj_z, pose_conf = getObjectPosition(center[0], center[1], img_h, img_w, dist_from_camera=dist_from_camera)

                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 
                    obj_theta_z.append(theta_z)
                    pose_confidence.append(pose_conf) 

                    symbolOnLeft = analyzeGate(cropToBbox(img, bbox), debug_img)
                    extra_field.append(symbolOnLeft) # 1 for earth, 0 for the other one
                elif global_class_id == 2: # BUOY
                    theta_z = measureBuoyAngle(depth_cropped)
                    pred_obj_x, pred_obj_y, pred_obj_z, pose_conf = getObjectPosition(center[0], center[1], img_h, img_w, dist_from_camera=dist_from_camera)
                    
                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 
                    obj_theta_z.append(theta_z)
                    pose_confidence.append(pose_conf) 
                    extra_field.append(None)

                    buoy_symbols = analyzeBuoy(cropToBbox(img, bbox), debug_img)
                    for symbol_conf, symbol_x, symbol_y, symbol_z, symbol_pose_conf, symbol_priority in buoy_symbols:
                        label.append(4)
                        detection_confidence.append(symbol_conf)
                        obj_x.append(symbol_x)
                        obj_y.append(symbol_y)
                        obj_z.append(symbol_z) 
                        obj_theta_z.append(theta_z)
                        pose_confidence.append(symbol_pose_conf) 
                        extra_field.append(symbol_priority)

    #create object detection frame message and publish it
    detectionFrame = ObjectDetectionFrame()
    detectionFrame.label = label
    detectionFrame.detection_confidence = detection_confidence
    detectionFrame.x = obj_x
    detectionFrame.y = obj_y
    detectionFrame.z = obj_z
    detectionFrame.theta_z = obj_theta_z
    detectionFrame.pose_confidence = pose_confidence
    detectionFrame.extra_field = extra_field
    pub.publish(detectionFrame)
    #convert visualization image to sensor_msg image and publish it to corresponding cameras visualization topic
    img = bridge.cv2_to_imgmsg(img, "bgr8")
    visualisation_pubs[camera_id].publish(img)

lane_marker_z = -3.7
octagon_z = 0

if __name__ == '__main__':
    rospy.init_node('object_detection')

    detect_every = 5  #run the model every _ frames received (to not eat up too much RAM)
    #only report predictions with confidence at least 40%
    min_prediction_confidence = 0.4
    
    pwd = os.path.realpath(os.path.dirname(__file__))
    down_cam_model_filename = pwd + "/models/down_cam_model.pt"
    quali_model_filename = pwd + "/models/quali_model.pt"
    model = [
        YOLO(down_cam_model_filename),
        YOLO(quali_model_filename)
        ]
    for m in model: m.to(torch.device('cuda'))
    #count for number of images received per camera
    i = [
        0,
        0
        ]
    class_names = [ #one array per camera, name index should be class id
        ["Lane Marker", "Octagon"],
        ["Lane Marker", "Gate", "Buoy"],
        ]
    global_class_ids = {"Lane Marker":0, "Gate":1, "Buoy":2, "Octagon":3, "Buoy Symbol":4}
    #the int argument is used to index debug publisher, model, class names, and i
    subs = [
        rospy.Subscriber('/vision/down_cam/image_raw', Image, detect_on_image, 0),
        rospy.Subscriber('/vision/front_cam/image_rgb', Image, detect_on_image, 1)
        ]

    pub = rospy.Publisher('vision/viewframe_detection', ObjectDetectionFrame, queue_size=1)
    rospy.spin()
