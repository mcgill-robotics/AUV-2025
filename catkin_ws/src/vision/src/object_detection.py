#!/usr/bin/env python3

import numpy as np
import rospy
from auv_msgs.msg import VisionObject, VisionObjectArray
from object_detection_utils import *
import object_detection_utils
import torch

# structurally good
# dont forget to change things here after modifying object_detection_utils.py
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
            states[camera_id].resume()
            return
    if camera_id == 1 and states[camera_id].point_cloud is None:
        print("Point cloud not yet published.")
        states[camera_id].resume()
        return
    
    #convert image to cv2
    img = bridge.imgmsg_to_cv2(raw_img, "bgr8")
    debug_img = np.copy(img)
    
    #run model on img
    detections = model[camera_id].predict(img, device=device, verbose=print_debug_info) #change device for cuda

    #initialize empty array for object detection frame message
    detectionFrameArray = []
    img_h, img_w, _ = img.shape
    if camera_id == 1:
        #[COMP] change target symbol to match planner

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
                if print_debug_info: print("Confidence too low for camera {} ({}%)".format(camera_id, conf*100))
                continue
            
            bbox = list(box.xywh[0])
            cls_id = int(list(box.cls)[0])
            global_class_name = class_names[camera_id][cls_id]
            #add bbox visualization to img
            debug_img = visualizeBbox(debug_img, bbox, global_class_name + " " + str(conf*100) + "%")

            #Create a new detection frame object
            detectionFrame = VisionObject()
            if camera_id == 0: # DOWN CAM
                if global_class_name == "Lane Marker":
                    detectionFrame.label = global_class_name
                    detectionFrame.confidence = conf
                    headings, center, debug_img = measureLaneMarker(img, bbox, debug_img)
                    if None in headings:
                        detectionFrame.extra_field = None
                        detectionFrame.theta_z = None
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
                        detectionFrame.theta_z = heading1 + down_cam_yaw_offset
                        detectionFrame.extra_field = heading2 + down_cam_yaw_offset

                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPositionDownCam(bbox[0], bbox[1], img_h, img_w, lane_marker_top_z)
                    detectionFrame.x = pred_obj_x
                    detectionFrame.y = pred_obj_y
                    detectionFrame.z = pred_obj_z

                    # Add the detection frame to the array
                    detectionFrameArray.append(detectionFrame)
                    
                elif global_class_name == "Octagon Table": # OCTAGON TABLE
                    detectionFrame.label = global_class_name
                    detectionFrame.confidence = conf
                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPositionDownCam(bbox[0], bbox[1], img_h, img_w, octagon_table_top_z)
                    detectionFrame.x = pred_obj_x
                    detectionFrame.y = pred_obj_y
                    detectionFrame.z = pred_obj_z
                    detectionFrame.extra_field = None
                    detectionFrame.theta_z = None

                    # Add the detection frame to the array
                    detectionFrameArray.append(detectionFrame)

                
            else: # FORWARD CAM
                if global_class_name == "Lane Marker" or global_class_name == "Octagon Table": # LANE MARKER OR OCTAGON TABLE
                    detectionFrame.label = global_class_name
                    detectionFrame.confidence = conf
                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPositionFrontCam(bbox)
                    detectionFrame.x = pred_obj_x
                    detectionFrame.y = pred_obj_y
                    detectionFrame.z = pred_obj_z
                    detectionFrame.theta_z = None
                    detectionFrame.extra_field = None

                    detectionFrameArray.append(detectionFrame)
                elif global_class_name == "Gate": # GATE
                    detectionFrame.label = global_class_name
                    detectionFrame.confidence = conf
                    theta_z = measureAngle(bbox)
                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPositionFrontCam(bbox)
                    detectionFrame.x = pred_obj_x
                    detectionFrame.y = pred_obj_y
                    detectionFrame.z = pred_obj_z
                    detectionFrame.theta_z = theta_z
                    detectionFrame.extra_field = leftmost_gate_symbol # 1 for earth, 0 for the other one

                    detectionFrameArray.append(detectionFrame)
                elif global_class_name == "Buoy": # BUOY
                    detectionFrame.label = global_class_name
                    detectionFrame.confidence = conf
                    theta_z = measureAngle(bbox)
                    pred_obj_x, pred_obj_y, pred_obj_z = getObjectPositionFrontCam(bbox)
                    detectionFrame.x = pred_obj_x
                    detectionFrame.y = pred_obj_y
                    detectionFrame.z = pred_obj_z
                    detectionFrame.theta_z = theta_z
                    detectionFrame.extra_field = None

                    detectionFrameArray.append(detectionFrame)

                    for symbol_class_name, symbol_x, symbol_y, symbol_z, symbol_confidence in buoy_symbols:
                        detectionFrame = VisionObject()
                        detectionFrame.label = symbol_class_name
                        detectionFrame.x = symbol_x
                        detectionFrame.y = symbol_y
                        detectionFrame.z = symbol_z
                        detectionFrame.confidence = symbol_confidence
                        detectionFrame.theta_z = theta_z
                        detectionFrame.extra_field = None

                        detectionFrameArray.append(detectionFrame)

                    
                
    for obj in detectionFrameArray:
        obj.x = obj.x if obj.x is not None else -1234.5 
        obj.theta_z = obj.theta_z if obj.theta_z is not None else -1234.5
        obj.extra_field = obj.extra_field if obj.extra_field is not None else -1234.5
    

    detectionFrameArray = cleanDetections(detectionFrameArray)

    if len(detectionFrameArray) > 0:
        #create object detection frame message and publish it
        detectionFrameArrayMsg = VisionObjectArray()
        detectionFrameArrayMsg.array = detectionFrameArray
        pub.publish(detectionFrameArrayMsg)
        
    #convert visualization image to sensor_msg image and publish it to corresponding cameras visualization topic
    debug_img = bridge.cv2_to_imgmsg(debug_img, "bgr8")
    visualisation_pubs[camera_id].publish(debug_img)   
    states[camera_id].resume()

print_debug_info = rospy.get_param("log_model_prediction_info", False)

#the int argument is used to index debug publisher, model, class names, and i
subs = [
    rospy.Subscriber('/vision/down_cam/image_raw', Image, detect_on_image, 0),
    rospy.Subscriber('vision/front_cam/color/image_raw', Image, detect_on_image, 1)
    ]
rospy.spin()
