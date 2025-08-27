#!/usr/bin/env python3

import rospy
import numpy as np

import torch
import ast
import quaternion


# Critical: Import cv2 and related libs BEFORE sklearn (even indirectly)
import cv2
from cv_bridge import CvBridge

# Now it's safe to import modules that may include sklearn (like object_detection_utils)
from object_detection_utils import *  # Only after cv2

# Vision/deep learning tools
from ultralytics import YOLO
from lane_marker_measure import measure_lane_marker

from vision_state import VisionState

# ROS messages
from auv_msgs.msg import VisionObject, VisionObjectArray
from std_msgs.msg import Int32MultiArray, Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

def is_vision_ready(camera_id):
    # Only predict if cameras_image_count has not reached DETECT_EVERY yet.
    global cameras_image_count
    global states
    cameras_image_count[camera_id] += 1
    if cameras_image_count[camera_id] <= DETECT_EVERY:
        return False
    # Reset cameras_image_count.
    cameras_image_count[camera_id] = 0

    #the bloc that is commented out below hasn't been tested
    # states[camera_id].pause()
    # current_states = {
    #     "position": states[camera_id].position,
    #     "q": states[camera_id].q_auv,
    #     "theta_z": states[camera_id].theta_z,
    # }
    # for v in current_states.values():
    #     if v is None:
    #         print("State information missing. Skipping detection.")
    #         print(current_states)
    #         states[camera_id].resume()
    #         return False
    # if camera_id == 1 and states[camera_id].point_cloud is None:
    #     print("Point cloud not yet published.")
    #     states[camera_id].resume()
    #     return False
    return True


def handle_detections(image: np.ndarray, detections, camera_id) -> np.ndarray:
    """
    Handles post-processing of Ultralytics image detections by mapping them over the depth-map and
    extracting a point_cloud

    Args:
        image: Raw OpenCV image on which predictions were made
        detections: Detection array coming from any Vision Model
        camera_id: ID of the camera the detections were performed on
    
    Returns:
        Image with bounding boxes drawn over

    """
    global states
    
    # Initialize empty array for object detection frame message.
    detection_frame_array = []
    image_h, image_w, _ = image.shape

    depth_img_msg: Image = rospy.wait_for_message(
        "/vision/front_cam/aligned_depth_to_color/image_raw", Image)
    depth_img = bridge.imgmsg_to_cv2(depth_img_msg, desired_encoding="32FC1")

    # Nested for loops get all predictions made by model.
    for detection in detections:
        boxes = (
            detection.boxes.cpu().numpy()
            if is_cuda_available
            else detection.boxes.numpy()
        )

        for box in boxes:
            conf = float(list(box.conf)[0])

            if conf < MIN_PREDICTION_CONFIDENCE:
                continue

            x, y, w, h = list(box.xywh[0])
            cls_id = int(box.cls[0])
            global_class_name = class_names[camera_id][cls_id]
            if global_class_name not in ["sawfish", "shark", "gate_divider"]:
                continue

            # --- Visualize the bounding boxes onto the image before publishing
            cx, cy, bw, bh = float(x), float(y), float(w), float(h)
            x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
            x2, y2 = int(cx + bw / 2), int(cy + bh / 2)
            x1 = max(0, min(x1, image_w - 1))
            y1 = max(0, min(y1, image_h - 1))
            x2 = max(0, min(x2, image_w - 1))
            y2 = max(0, min(y2, image_h - 1))

            rng    = np.random.default_rng(hash(global_class_name) & 0xFFFFFFFF)
            colour = tuple(int(c) for c in rng.integers(0, 255, size=3))

            cv2.rectangle(image, (x1, y1), (x2, y2), colour, 2)
            label_txt = f"{global_class_name}: {conf:.2f}"
            (txt_w, txt_h), bl = cv2.getTextSize(label_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(image, (x1, y1 - txt_h - bl), (x1 + txt_w, y1), colour, cv2.FILLED)
            cv2.putText(image, label_txt, (x1, y1 - bl), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1, cv2.LINE_AA)
            # --- End of Visualization

            # Initialize a new detection frame object.
            detectionFrame = VisionObject()
            pred_obj_x = pred_obj_y = pred_obj_z = 0
            extra_field = theta_z = None

            # Post process by adding to the point cloud 
            if global_class_name not in ["sawfish", "shark", "gate_divider"]:
                continue

            if global_class_name is "gate_divider":
                theta_z = measure_angle(box)
                
            # Calculate the mean depth over the depth image
            roi = depth_img[y1:y2, x1:x2]
            roi = roi[~np.isnan(roi)] 
            roi = roi[roi > 0]  
            mean_depth = np.nanmedian(roi) if roi.size else np.inf

            # Rotate based on the local to world frame difference
            curr_state = states[camera_id]
            
            global_obj_pos_offset = quaternion.rotate_vectors(
                curr_state.q_auv, np.array([x, y, mean_depth])
            )
            global_x, global_y, global_z = global_obj_pos_offset + np.array(
                [curr_state.position.x, curr_state.position.y, curr_state.position.z]
            )

            # Set detection frame up for publishing
            detectionFrame.label = global_class_name
            detectionFrame.x = global_x
            detectionFrame.y = global_y
            detectionFrame.z = global_z
            detectionFrame.theta_z = theta_z
            detectionFrame.extra_field = extra_field
            detectionFrame.confidence = conf * calculate_bbox_confidence(
                list(box.xywh[0]), image_h, image_w
            )

            # Add the detection frame to the array.
            detection_frame_array.append(detectionFrame)

    publish_detection_frame(detection_frame_array)
    return image

def publish_detection_frame(detection_frame_array) -> None:
    """
    Publishes an array of detection frames into ROS topics

    Args:
        detection_frame_array: List of DetectionFrames

    Returns:
        None
    """
    for obj in detection_frame_array:
        obj.x = obj.x if obj.x is not None else NULL_PLACEHOLDER
        obj.theta_z = obj.theta_z if obj.theta_z is not None else NULL_PLACEHOLDER
        obj.extra_field = (
            obj.extra_field if obj.extra_field is not None else NULL_PLACEHOLDER
        )

    if len(detection_frame_array) > 0:
        # Create object detection frame message and publish it.
        detection_frame_array = clean_detections(detection_frame_array)
        detection_frame_arrayMsg = VisionObjectArray()
        detection_frame_arrayMsg.array = detection_frame_array
        pub_viewframe_detection.publish(detection_frame_arrayMsg)


def vision_cb(raw_image: Image, camera_id: int) -> None:
    """
    Processes raw image streams on a specific camera, and publishes object detection messages using
    a global pubs_visualization array. 
    
    Keyword arguments:
        raw_Image -- RGB-D colorized raw image from a camera
        camera_id -- id of the camera the callback function operates on
    Return: None
    """
    global states

    if not is_vision_ready(camera_id):
        return

    # Convert image to cv2.
    try:
        image = bridge.imgmsg_to_cv2(raw_image, "bgr8")
    except Exception as e:
        rospy.logerr(f"cv_bridge conversion failed: {e}")

    # Copy the OpenCV image type to a stateful store globally in the node
    states[camera_id].bgr_image = np.copy(image)

    detection_results = model[camera_id].predict(
        image, device=device, verbose=PRINT_DEBUG_INFO
    )
    detection_img : np.ndarray = np.copy(
        handle_detections(image, detection_results, camera_id)
    )
    # TODO: Add an alternative segmentation model that runs parallel

    # Convert OpenCV image type back to ROS msg type to be published
    detection_img_msg : Image = bridge.cv2_to_imgmsg(detection_img, "bgr8")
    pubs_visualisation[camera_id].publish(detection_img_msg)
    states[camera_id].resume()

if __name__ == "__main__":
    rospy.init_node("object_detection", anonymous=True)

    PRINT_DEBUG_INFO = rospy.get_param("log_model_prediction_info", False)
    NULL_PLACEHOLDER = rospy.get_param("NULL_PLACEHOLDER")

    MIN_PREDICTION_CONFIDENCE = rospy.get_param("min_prediction_confidence")

    POOL_DEPTH = rospy.get_param("pool_depth")
    OCTAGON_TABLE_HEIGHT = rospy.get_param("octagon_table_height")
    LANE_MARKER_HEIGHT = rospy.get_param("lane_marker_height")
    BIN_HEIGHT = rospy.get_param("bin_height")
    lane_marker_top_z = POOL_DEPTH + LANE_MARKER_HEIGHT
    octagon_table_top_z = POOL_DEPTH + OCTAGON_TABLE_HEIGHT
    bin_top_z = POOL_DEPTH + BIN_HEIGHT
    DOWN_CAM_YAW_OFFSET = rospy.get_param("down_cam_yaw_offset")

    # Run the model every _ frames received (to not eat up too much RAM).
    DETECT_EVERY = rospy.get_param("object_detection_frame_interval")

    DOWN_CAM_MODEL_FILE = rospy.get_param("down_cam_model_file")
    FRONT_CAM_MODEL_FILE = rospy.get_param("front_cam_model_file")

    model = [YOLO(DOWN_CAM_MODEL_FILE), YOLO(FRONT_CAM_MODEL_FILE)]

    is_cuda_available = torch.cuda.is_available()
    if not is_cuda_available:
        rospy.logwarn("CUDA is not available! YOLO inference will run on CPU.")
        device = "cpu"
    else:
        device = 0
        for m in model:
            m.to(torch.device("cuda"))

    # One array per camera, name index should be class id.
    class_names = [
        ast.literal_eval(rospy.get_param("down_cam_class_name_mappings")),
        ast.literal_eval(rospy.get_param("front_cam_class_name_mappings")),
    ]

    # Count for number of images received per camera.
    cameras_image_count = [0, 0]

    pubs_visualisation = [
        rospy.Publisher("/vision/down_cam/detection", Image, queue_size=1),
        rospy.Publisher("/vision/front_cam/detection", Image, queue_size=1),
    ]
    pub_viewframe_detection = rospy.Publisher(
        "/vision/viewframe_detection", VisionObjectArray, queue_size=1
    )
    pub_bbox_centering = rospy.Publisher("/vision/down_cam/bbox", Int32MultiArray, queue_size=1)

    bridge = CvBridge()

    # The int argument is used to index debug publisher, model, class names, and cameras_image_count.
    # rospy.Subscriber("/vision/down_cam/image_raw", Image, vision_cb, 0),
    rospy.Subscriber("/zed2i/zed_node/stereo/image_rect_color", Image, vision_cb, 1),

    rospy.spin()
