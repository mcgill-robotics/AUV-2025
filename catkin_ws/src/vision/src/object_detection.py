#!/usr/bin/env python3

import numpy as np
import rospy
import os
from ultralytics import YOLO
from sensor_msgs.msg import Image
from auv_msgs.msg import ObjectDetectionFrame
from object_detection_utils import *
import object_detection_utils
import torch

#callback when an image is received
#runs model on image, publishes detection frame and generates/publishes visualization of predictions
def detect_on_image(raw_img, camera_id):
    #only predict if i has not reached detect_every yet
    global i
    i[camera_id] += 1
    if i[camera_id] <= detect_every: return
    i[camera_id] = 0
    states[camera_id].pause()
    
    current_states = {"x:": states[camera_id].x, "y:": states[camera_id].y, "z": states[camera_id].z, "theta_x": states[camera_id].theta_x, "theta_y": states[camera_id].theta_y, "theta_z": states[camera_id].theta_z, "depth": states[camera_id].depth_map}
    for v in current_states.values():
        if v is None:
            print("State information missing. Skipping detection.")
            print(current_states)
            return
    #convert image to cv2
    img = bridge.imgmsg_to_cv2(raw_img, "bgr8")
    debug_img = np.copy(img)
    
    #run model on img
    detections = model[camera_id].predict(img, device=device) #change device for cuda
    #initialize empty arrays for object detection frame message
    label = []
    obj_x = []
    obj_y = []
    obj_z = []
    obj_theta_z = []
    extra_field = []
    confidences = []
    if camera_id == 1:
        buoy_symbols = []
        buoy_symbols = analyzeBuoy(detections, min_prediction_confidence, class_names[1].index("Earth Symbol"), class_names[1].index("Abydos Symbol"), class_names[1].index("Buoy"), states[camera_id].depth_map)
        leftmost_gate_symbol = analyzeGate(detections, min_prediction_confidence, class_names[1].index("Earth Symbol"), class_names[1].index("Abydos Symbol"), class_names[1].index("Gate"))
    #nested for loops get all predictions made by model
    for detection in detections:
        if torch.cuda.is_available(): boxes = detection.boxes.cpu().numpy()
        else: boxes = detection.boxes.numpy()
        for box in boxes:
            conf = float(list(box.conf)[0])
            #only consider prediction if confidence is at least min_prediction_confidence
            if conf < min_prediction_confidence:
                print("confidence too low ({}%)".format(conf*100))
                continue
            
            bbox = list(box.xywh[0])
            cls_id = int(list(box.cls)[0])
            global_class_id = global_class_ids[class_names[camera_id][cls_id]]
            #add bbox visualization to img
            debug_img = visualizeBbox(debug_img, bbox, class_names[camera_id][cls_id] + " " + str(conf*100) + "%")
            img_h, img_w, _ = img.shape
            if camera_id == 0: # DOWN CAM
                if global_class_id == 0: #LANE MARKER
                    label.append(global_class_id)
                    confidences.append(conf)
                    headings, center, debug_img = measureLaneMarker(img, bbox, debug_img)
                    if None in headings:
                        extra_field.append(None)
                        obj_theta_z.append(None)
                        center = bbox
                    else:
                        if headings[0] < -90:
                            heading1 = states[camera_id].theta_z + (headings[0]+270)
                        else:
                            heading1 = states[camera_id].theta_z + (headings[0]-90)
                        if headings[1] < -90:
                            heading2 = states[camera_id].theta_z + (headings[1]+270)
                        else:
                            heading2 = states[camera_id].theta_z + (headings[1]-90)
                        if heading1 > heading2:
                            obj_theta_z.append(heading1)
                            extra_field.append(heading2)
                        else:
                            obj_theta_z.append(heading2)
                            extra_field.append(heading1)

                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPosition(center[0], center[1], img_h, img_w, z_pos=lane_marker_z)
                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 
                elif global_class_id == 3: # OCTAGON TABLE
                    label.append(global_class_id)
                    confidences.append(conf)
                    center = bbox
                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPosition(bbox[0], bbox[1], img_h, img_w, z_pos=octagon_table_z)
                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 
                    extra_field.append(None)
                    obj_theta_z.append(None)
            else: # FORWARD CAM
                center = bbox
                dist_from_camera = object_depth(cropToBbox(states[camera_id].depth_map, bbox), global_class_id)
                if global_class_id == 0: # LANE MARKER
                    label.append(global_class_id)
                    confidences.append(conf)
                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPosition(center[0], center[1], img_h, img_w, dist_from_camera=dist_from_camera)
                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 
                    obj_theta_z.append(None)
                    extra_field.append(None)
                elif global_class_id == 1: # GATE
                    label.append(global_class_id)
                    confidences.append(conf)
                    theta_z = measureGateAngle(states[camera_id].depth_map, gate_width, bbox)
                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPosition(center[0], center[1], img_h, img_w, dist_from_camera=dist_from_camera)
                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 
                    obj_theta_z.append(theta_z)
                    extra_field.append(leftmost_gate_symbol) # 1 for earth, 0 for the other one
                elif global_class_id == 2: # BUOY
                    label.append(global_class_id)
                    confidences.append(conf)
                    theta_z = measureBuoyAngle(states[camera_id].depth_map, buoy_width, bbox)
                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPosition(center[0], center[1], img_h, img_w, dist_from_camera=dist_from_camera)
                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 
                    obj_theta_z.append(theta_z)
                    extra_field.append(None)

                    for symbol_class_id, symbol_x, symbol_y, symbol_z, confidence in buoy_symbols:
                        label.append(symbol_class_id)
                        obj_x.append(symbol_x)
                        obj_y.append(symbol_y)
                        obj_z.append(symbol_z) 
                        confidences.append(confidence)
                        obj_theta_z.append(theta_z)
                        extra_field.append(None)
                elif global_class_id == 3: # OCTAGON TABLE
                    label.append(global_class_id)
                    confidences.append(conf)
                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPosition(center[0], center[1], img_h, img_w, dist_from_camera=dist_from_camera)
                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 
                    obj_theta_z.append(None)
                    extra_field.append(None)

    extra_field = [x if not x is None else -1234.5 for x in extra_field]
    obj_theta_z = [x if not x is None else -1234.5 for x in obj_theta_z]

    label, obj_x, obj_y, obj_z, obj_theta_z, extra_field = cleanDetections(label, obj_x, obj_y, obj_z, obj_theta_z, extra_field, confidences, max_counts_per_label)

    #create object detection frame message and publish it
    detectionFrame = ObjectDetectionFrame()
    detectionFrame.label = label
    detectionFrame.x = obj_x
    detectionFrame.y = obj_y
    detectionFrame.z = obj_z
    detectionFrame.theta_z = obj_theta_z
    detectionFrame.extra_field = extra_field
    pub.publish(detectionFrame)
    #convert visualization image to sensor_msg image and publish it to corresponding cameras visualization topic
    debug_img = bridge.cv2_to_imgmsg(debug_img, "bgr8")
    visualisation_pubs[camera_id].publish(debug_img)
    states[camera_id].resume()

pool_depth = -4
lane_marker_z = pool_depth + 0.3
octagon_table_z = pool_depth + 1.0668
buoy_width = 1.22 
gate_width = 3

if __name__ == '__main__':
    detect_every = 5  #run the model every _ frames received (to not eat up too much RAM)
    #only report predictions with confidence at least 40%
    min_prediction_confidence = 0.4
    
    pwd = os.path.realpath(os.path.dirname(__file__))
    # down_cam_model_filename = pwd + "/models/down_cam_model.pt"
    # gate_model_filename = pwd + "/models/front_cam_model.pt"
    down_cam_model_filename = pwd + "/models/down_cam_model_sim.pt"
    gate_model_filename = pwd + "/models/front_cam_sim.pt"
    model = [
        YOLO(down_cam_model_filename),
        YOLO(gate_model_filename)
        ]
    for m in model:
        if torch.cuda.is_available(): m.to(torch.device('cuda'))
        else: print("WARN: CUDA is not available! Running on CPU")
    
    if torch.cuda.is_available(): device=0
    else: device = 'cpu'
    #count for number of images received per camera
    i = [
        0,
        0
        ]
    class_names = [ #one array per camera, name index should be class id
        ["Lane Marker", "Octagon Table"],
        ["Abydos Symbol", "Buoy", "Earth Symbol", "Gate", "Lane Marker", "Octagon", "Octagon Table"],
        ]
    global_class_ids = {"Lane Marker":0, "Gate":1, "Buoy":2, "Octagon Table":3, "Earth Symbol":4, "Abydos Symbol":5, "Octagon":6}
    #NOTE: WHEN UPDATING ALSO UPDATE object_depth() IN OBJECT_DETECTION_UTILS!

    max_counts_per_label = [1, 1, 1, 1, 2, 2, 1]

    pub = rospy.Publisher('vision/viewframe_detection', ObjectDetectionFrame, queue_size=1)

    #the int argument is used to index debug publisher, model, class names, and i
    subs = [
        rospy.Subscriber('/vision/down_cam/image_raw', Image, detect_on_image, 0),
        rospy.Subscriber('/vision/front_cam/image_rgb', Image, detect_on_image, 1)
        ]
    
    rospy.spin()
