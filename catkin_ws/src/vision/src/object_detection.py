#!/usr/bin/env python3

import numpy as np
import rospy
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
    
    current_states = {"x:": states[camera_id].x, "y:": states[camera_id].y, "z": states[camera_id].z, "theta_x": states[camera_id].theta_x, "theta_y": states[camera_id].theta_y, "theta_z": states[camera_id].theta_z}
    for v in current_states.values():
        if v is None:
            print("State information missing. Skipping detection.")
            print(current_states)
            return
    if camera_id == 1 and states[camera_id].point_cloud is None:
        print("Point cloud not yet published.")
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
        buoy_symbols = analyzeBuoy(detections)
        leftmost_gate_symbol = analyzeGate(detections)
    #nested for loops get all predictions made by model
    for detection in detections:
        if torch.cuda.is_available(): boxes = detection.boxes.cpu().numpy()
        else: boxes = detection.boxes.numpy()
        for box in boxes:
            conf = float(list(box.conf)[0])
            #only consider prediction if confidence is at least min_prediction_confidence
            if conf < min_prediction_confidence:
                print("Confidence too low for camera {} ({}%)".format(camera_id, conf*100))
                continue
            
            bbox = list(box.xywh[0])
            cls_id = int(list(box.cls)[0])
            global_class_name = class_names[camera_id][cls_id]
            #add bbox visualization to img
            debug_img = visualizeBbox(debug_img, bbox, global_class_name + " " + str(conf*100) + "%")
            img_h, img_w, _ = img.shape
            if camera_id == 0: # DOWN CAM
                if global_class_name == "Lane Marker":
                    label.append(global_class_name)
                    confidences.append(conf)
                    headings, center, debug_img = measureLaneMarker(img, bbox, debug_img)
                    if None in headings:
                        extra_field.append(None)
                        obj_theta_z.append(None)
                    else:
                        bbox = center
                        if headings[0] + down_cam_yaw_offset < -90:
                            heading1 = states[camera_id].theta_z + (headings[0]+270)
                        else:
                            heading1 = states[camera_id].theta_z + (headings[0]-90)
                        if headings[1] + down_cam_yaw_offset < -90:
                            heading2 = states[camera_id].theta_z + (headings[1]+270)
                        else:
                            heading2 = states[camera_id].theta_z + (headings[1]-90)
                        obj_theta_z.append(heading1 + down_cam_yaw_offset)
                        extra_field.append(heading2 + down_cam_yaw_offset)

                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPositionDownCam(bbox[0], bbox[1], img_h, img_w, lane_marker_z)
                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 
                elif global_class_name == "Octagon Table": # OCTAGON TABLE
                    label.append(global_class_name)
                    confidences.append(conf)
                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPositionDownCam(bbox[0], bbox[1], img_h, img_w, octagon_table_z)
                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 
                    extra_field.append(None)
                    obj_theta_z.append(None)
            else: # FORWARD CAM
                if global_class_name == "Lane Marker": # LANE MARKER
                    label.append(global_class_name)
                    confidences.append(conf)
                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPositionFrontCam(bbox)
                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 
                    obj_theta_z.append(None)
                    extra_field.append(None)
                elif global_class_name == "Gate": # GATE
                    label.append(global_class_name)
                    confidences.append(conf)
                    theta_z = measureAngle(bbox, global_class_name)
                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPositionFrontCam(bbox)
                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 
                    obj_theta_z.append(theta_z)
                    extra_field.append(leftmost_gate_symbol) # 1 for earth, 0 for the other one
                elif global_class_name == "Buoy": # BUOY
                    label.append(global_class_name)
                    confidences.append(conf)
                    theta_z = measureAngle(bbox, global_class_name)
                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPositionFrontCam(bbox)
                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 
                    obj_theta_z.append(theta_z)
                    extra_field.append(None)

                    for symbol_class_name, symbol_x, symbol_y, symbol_z, confidence in buoy_symbols:
                        label.append(symbol_class_name)
                        obj_x.append(symbol_x)
                        obj_y.append(symbol_y)
                        obj_z.append(symbol_z) 
                        confidences.append(confidence)
                        obj_theta_z.append(theta_z)
                        extra_field.append(None)
                elif global_class_name == "Octagon Table": # OCTAGON TABLE
                    label.append(global_class_name)
                    confidences.append(conf)
                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPositionFrontCam(bbox)
                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 
                    obj_theta_z.append(None)
                    extra_field.append(None)
                elif global_class_name == "Quali Gate": # QUALI GATE
                    label.append(global_class_name)
                    confidences.append(conf)
                    theta_z = measureAngle(bbox, global_class_name)
                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPositionFrontCam(bbox)
                    obj_x.append(pred_obj_x)
                    obj_y.append(pred_obj_y)
                    obj_z.append(pred_obj_z) 
                    obj_theta_z.append(theta_z)
                    extra_field.append(None)

    extra_field = [x if not x is None else -1234.5 for x in extra_field]
    obj_theta_z = [x if not x is None else -1234.5 for x in obj_theta_z]

    label, obj_x, obj_y, obj_z, obj_theta_z, extra_field = cleanDetections(label, obj_x, obj_y, obj_z, obj_theta_z, extra_field, confidences)

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


#the int argument is used to index debug publisher, model, class names, and i
subs = [
    rospy.Subscriber('/vision/down_cam/image_raw', Image, detect_on_image, 0),
    rospy.Subscriber('vision/front_cam/color/image_raw', Image, detect_on_image, 1)
    ]
rospy.spin()
