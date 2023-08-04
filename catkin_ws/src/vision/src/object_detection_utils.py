import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from ultralytics import YOLO
from auv_msgs.msg import ObjectDetectionFrame
import numpy as np
import math
from std_msgs.msg import Float64
import lane_marker_measure
import torch
from geometry_msgs.msg import Pose
import quaternion
import os
from tf import transformations
class State:
    def __init__(self, isFrontCamState):
        self.x = None
        self.y = None
        self.z = None
        self.theta_x = None
        self.theta_y = None
        self.theta_z = None
        self.paused = False
        self.q_auv = None

        self.x_pos_sub = rospy.Subscriber('state_x', Float64, self.updateX)
        self.y_pos_sub = rospy.Subscriber('state_y', Float64, self.updateY)
        self.z_pos_sub = rospy.Subscriber('state_z', Float64, self.updateZ)
        self.pose_sub = rospy.Subscriber('pose', Pose, self.updatePose)
        self.theta_x_sub = rospy.Subscriber('state_theta_x', Float64, self.updateThetaX)
        self.theta_y_sub = rospy.Subscriber('state_theta_y', Float64, self.updateThetaY)
        self.theta_z_sub = rospy.Subscriber('state_theta_z', Float64, self.updateThetaZ)
    def updateX(self, msg):
        if self.paused: return
        self.x = float(msg.data)
    def updateY(self, msg):
        if self.paused: return
        self.y = float(msg.data)
    def updateZ(self, msg):
        if self.paused: return
        self.z = float(msg.data)
    def updateThetaX(self, msg):
        if self.paused: return
        self.theta_x = float(msg.data)
    def updateThetaY(self, msg):
        if self.paused: return
        self.theta_y = float(msg.data)
    def updateThetaZ(self, msg):
        if self.paused: return
        self.theta_z = float(msg.data)
    def updatePose(self,msg):
        if self.paused: return
        self.q_auv = np.quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
    def pause(self):
        self.paused = True
    def resume(self):
        self.paused = False

#given an image, class name, and a bounding box, draws the bounding box rectangle and label name onto the image
def visualizeBbox(img, bbox, class_name, thickness=2, fontSize=0.5):
    #get xmin, xmax, ymin, ymax from bbox 
    x_center, y_center, w, h = bbox
    x_min = x_center - w/2
    y_min = y_center - h/2
    x_min, x_max, y_min, y_max = int(x_min), int(x_min + w), int(y_min), int(y_min + h)
    #draw bounding box on image
    cv2.rectangle(img, (x_min, y_min), (x_max, y_max), color=BOX_COLOR, thickness=thickness)
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
        lineType=cv2.LINE_AA)
    return img

#given a bounding box and image, returns the image cropped to the bounding box (to isolate detected objects)
def cropToBbox(img, bbox, copy=True):
    x_center, y_center, w, h = bbox
    x_min = x_center - w/2
    y_min = y_center - h/2
    x_min, x_max, y_min, y_max = int(x_min), int(x_min + w), int(y_min), int(y_min + h)
    if copy: return np.copy(img[y_min:y_max, x_min:x_max])
    else: return img[y_min:y_max, x_min:x_max]

def measureLaneMarker(img, bbox, debug_img):
    #crop image to lane marker
    cropped_img = cropToBbox(img, bbox)
    line_thickness = 2 # in pixels
    line_length = 0.25*min(bbox[2], bbox[3]) #line will be size of shortest bounding box side
    #measure headings from lane marker
    cropped_img_to_pub = bridge.cv2_to_imgmsg(cropped_img, "bgr8")
    cropped_img_pubs[0].publish(cropped_img_to_pub)
    headings, center_point = lane_marker_measure.measure_headings(cropped_img, debug_img=cropToBbox(debug_img, bbox, copy=False))
    if None in (headings, center_point): return (None, None), (None, None), debug_img
    center_point_x = center_point[0] + bbox[0] - bbox[2]/2
    center_point_y = center_point[1] + bbox[1] - bbox[3]/2
    center_point = (int(center_point_x), int(center_point_y))
    for angle in headings:
        #get angle, line start and line end from heading slope
        slope = math.tan((angle/-180)*math.pi)
        #calculate line x length from total length
            #line_length = sqrt(line_x_length**2 + line_y_length**2)
            #line_length**2 = line_x_length**2 + (line_x_length*slope)**2
            #line_length**2 = line_x_length**2 * (1 + slope**2)
            #line_x_length = sqrt(line_length**2 / (1 + slope**2))
        line_x_length = math.sqrt((line_length ** 2) / (1 + slope ** 2))
        if abs(angle) > 90: #heading goes into negative x
            line_end = (int(center_point[0]-line_x_length), int(center_point[1] - slope*line_x_length)) # (x,y)
        else: # heading goes into positive x
            line_end = (int(center_point[0]+line_x_length), int(center_point[1] + slope*line_x_length)) # (x,y)
        #draw line on image
        cv2.line(debug_img, center_point, line_end, HEADING_COLOR, line_thickness)
        #add text with measured angle of line at the end of the line
        cv2.putText(
            debug_img,
            text=str(angle) + " deg.",
            org=line_end,
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.4, 
            color=HEADING_COLOR, 
            lineType=cv2.LINE_AA)
    cv2.circle(debug_img, center_point, radius=5, color=HEADING_COLOR, thickness=-1)
    return headings, center_point, debug_img

def transformLocalToGlobal(lx,ly,lz,camera_id,yaw_offset=0):
    rotation_offset = transformations.quaternion_from_euler(0, 0, yaw_offset)
    rotation = states[camera_id].q_auv * np.quaternion(rotation_offset[3], rotation_offset[0], rotation_offset[1], rotation_offset[2])
    return quaternion.rotate_vectors(rotation, np.array([lx,ly,lz])) + np.array([states[camera_id].x, states[camera_id].y, states[camera_id].z])

def eulerToVectorDownCam(x_deg, y_deg, z_diff):
    x_rad = math.radians(x_deg)
    y_rad = math.radians(y_deg)
    x = -math.tan(y_rad)
    y = math.tan(x_rad)
    vec = np.array([x,y,-1])
    vec = vec / np.linalg.norm(vec)
    return vec

def find_intersection(vector, plane_z_pos):
    if vector[2] == 0: return None
    scaling_factor = plane_z_pos / vector[2]
    if scaling_factor < 0: return None
    return np.array(vector) * scaling_factor

def eulerToVectorFrontCam(x_deg, y_deg, z_diff):
    x_rad = math.radians(x_deg)
    y_rad = math.radians(y_deg)
    y = math.tan(x_rad)
    z = -math.tan(y_rad)
    vec = np.array([1,y,z])
    vec = vec / np.linalg.norm(vec)
    return vec

def getObjectPositionDownCam(pixel_x, pixel_y, img_height, img_width, z_pos):
    #first calculate the relative offset of the object from the center of the image (i.e. map pixel coordinates to values from -0.5 to 0.5)
    x_center_offset = ((img_width/2) - pixel_x) / img_width #-0.5 to 0.5
    y_center_offset = (pixel_y - (img_height/2)) / img_height #negated since y goes from top to bottom
    #use offset within image and total FOV of camera to find an angle offset from the angle the camera is facing
    #assuming FOV increases linearly with distance from center pixel
    roll_angle_offset = down_cam_hfov*x_center_offset
    pitch_angle_offset = down_cam_vfov*y_center_offset

    local_direction_to_object = eulerToVectorDownCam(roll_angle_offset, pitch_angle_offset, z_pos - states[0].z)
    global_direction_to_object = transformLocalToGlobal(local_direction_to_object[0], local_direction_to_object[1], local_direction_to_object[2], camera_id=0, yaw_offset=down_cam_yaw_offset)

    # solve for point that is defined by the intersection of the direction to the object and it's z position
    obj_pos = find_intersection(global_direction_to_object, z_pos)
    if obj_pos is None or np.linalg.norm(obj_pos - np.array([states[0].x, states[0].y, states[0].z])) > max_dist_to_measure: return None, None, None
    x = obj_pos[0]
    y = obj_pos[1]
    z = z_pos
    return x, y, z

def estimateObjectPositionFrontCam(pixel_x, pixel_y, img_height, img_width, z_pos):

    #first calculate the relative offset of the object from the center of the image (i.e. map pixel coordinates to values from -0.5 to 0.5)
    x_center_offset = ((img_height/2) - pixel_x) / img_width #-0.5 to 0.5
    y_center_offset = (pixel_y - (img_height/2)) / img_height #negated since y goes from top to bottom
    #use offset within image and total FOV of camera to find an angle offset from the angle the camera is facing
    #assuming FOV increases linearly with distance from center pixel
    yaw_angle_offset = front_cam_hfov*x_center_offset
    pitch_angle_offset = front_cam_vfov*y_center_offset
    # TEST! NO IDEA IF THIS WORKS
    local_direction_to_object = eulerToVectorFrontCam(yaw_angle_offset, pitch_angle_offset)
    global_direction_to_object = transformLocalToGlobal(local_direction_to_object[0], local_direction_to_object[1], local_direction_to_object[2], 0)

    # solve for point that is defined by the intersection of the direction to the object and it's z position
    obj_pos = find_intersection(global_direction_to_object, z_pos)
    if obj_pos is None or np.linalg.norm(obj_pos - np.array([states[1].x, states[1].y, states[1].z])) > max_dist_to_measure: return np.nan, np.nan, np.nan
    x = obj_pos[0]
    y = obj_pos[1]
    z = z_pos
    return x, y, z
    
def getObjectPositionFrontCam(bbox, img_height, img_width, global_class_name, custom_z=None, custom_height=None):
    if global_class_name == "Gate":
        obj_center_z = gate_middle_z
        obj_height = gate_height
    elif global_class_name == "Buoy":
        obj_center_z = buoy_middle_z
        obj_height = buoy_height
    elif global_class_name == "Octagon Table":
        obj_center_z = octagon_table_top_z - octagon_table_height/2
        obj_height = octagon_table_height
    elif global_class_name == "Lane Marker": 
        obj_center_z = lane_marker_top_z - lane_marker_height/2
        obj_height = lane_marker_height 
    else:
        obj_center_z = custom_z
        obj_height = custom_height

    #TRY WITH DIFFERENT POINTS IN THE BBOX THAT WE KNOW THE Z POSITION OF:
    # NOTE: a later improvement could (maybe) be to use the corners as well (more complex, not sure if it will be more accurate or not)
    center = (bbox[0], bbox[1])
    top = (bbox[0], int(bbox[1] - bbox[3]/2))
    bottom = (bbox[0], int(bbox[1] + bbox[3]/2))

    center_x, center_y, center_z = estimateObjectPositionFrontCam(center[0], center[1], img_height, img_width, obj_center_z)

    top_x, top_y, top_z = estimateObjectPositionFrontCam(top[0], top[1], img_height, img_width, obj_center_z + obj_height/2)
    if top_z is not np.nan: top_z -= obj_height/2
    
    bottom_x, bottom_y, bottom_z = estimateObjectPositionFrontCam(bottom[0], bottom[1], img_height, img_width, obj_center_z - obj_height/2)
    if bottom_x is not np.nan: bottom_z += obj_height/2

    median_x = np.nanmedian(np.array([center_x, top_x, bottom_x]))
    median_y = np.nanmedian(np.array([center_y, top_y, bottom_y]))
    median_z = np.nanmedian(np.array([center_z, top_z, bottom_z]))

    if np.nan in [median_x, median_y, median_z]: return None, None, None

    return median_x, median_y, median_z


def measureAngle(bbox, img_height, img_width, global_class_name):
    if global_class_name == "Gate":
        obj_center_z = gate_middle_z
        obj_height = gate_height
    elif global_class_name == "Buoy":
        obj_center_z = buoy_middle_z
        obj_height = buoy_height
    else: 
        return None
        
    top_left = (int(bbox[0] - bbox[2]/2), int(bbox[1] - bbox[3]/2))
    left = (int(bbox[0] - bbox[2]/2), bbox[1])
    bottom_left = (int(bbox[0] - bbox[2]/2), int(bbox[1] + bbox[3]/2))

    top_right = (int(bbox[0] + bbox[2]/2), int(bbox[1] - bbox[3]/2))
    right = (int(bbox[0] + bbox[2]/2), bbox[1])
    bottom_right = (int(bbox[0] + bbox[2]/2), int(bbox[1] + bbox[3]/2))

    # GET LEFT POINT
    left_x, left_y, _ = estimateObjectPositionFrontCam(left[0], left[1], img_height, img_width, obj_center_z)

    top_left_x, top_left_y, top_left_z = estimateObjectPositionFrontCam(top_left[0], top_left[1], img_height, img_width, obj_center_z + obj_height/2)
    if top_left_z is not np.nan: top_left_z -= obj_height/2

    bottom_left_x, bottom_left_y, bottom_left_z = estimateObjectPositionFrontCam(bottom_left[0], bottom_left[1], img_height, img_width, obj_center_z - obj_height/2)
    if bottom_left_z is not np.nan: bottom_left_z += obj_height/2

    # TAKE THE MEASUREMENT WHICH IS IN THE MIDDLE OF THE OTHER TWO (TODO: FIND A BETTER WAY TO CHOOSE)
    median_left_x = np.nanmedian(np.array([left_x, top_left_x, bottom_left_x]))
    median_left_y = np.nanmedian(np.array([left_y, top_left_y, bottom_left_y]))

    left_point = [median_left_x, median_left_y]

    # GET RIGHT POINT
    right_x, right_y, _ = estimateObjectPositionFrontCam(right[0], right[1], img_height, img_width, obj_center_z)

    top_right_x, top_right_y, top_right_z = estimateObjectPositionFrontCam(top_right[0], top_right[1], img_height, img_width, obj_center_z + obj_height/2)
    if top_right_z is not np.nan: top_right_z -= obj_height/2

    bottom_right_x, bottom_right_y, bottom_right_z = estimateObjectPositionFrontCam(bottom_right[0], bottom_right[1], img_height, img_width, obj_center_z - obj_height/2)
    if bottom_right_z is not np.nan: bottom_right_z += obj_height/2

    # TAKE THE MEASUREMENT WHICH IS IN THE MIDDLE OF THE OTHER TWO (TODO: FIND A BETTER WAY TO CHOOSE)
    median_right_x = np.nanmedian(np.array([right_x, top_right_x, bottom_right_x]))
    median_right_y = np.nanmedian(np.array([right_y, top_right_y, bottom_right_y]))

    right_point = [median_right_x, median_right_y]

    if np.nan in right_point or np.nan in left_point: return None
    
    return measureYaw(left_point, right_point)


# returns an angle between a horizontal vector (i.e. a vector on the negative y axis) and the vector between a left and right (x,y) point on a gate or buoy
def measureYaw(leftPoint, rightPoint):
    zero_angle_vector = np.array([0,-1])
    magnitude_zero_vector = 1
    arg_vector = rightPoint - leftPoint
    magnitude_arg_vector = np.linalg.norm(arg_vector)
    dot_product = np.dot(zero_angle_vector, arg_vector)
    return math.acos(dot_product / (magnitude_zero_vector * magnitude_arg_vector)) * 180 / math.pi

def analyzeGate(detections):
    # Return the class_id of the symbol on the left of the gate
    # If no symbol return None
    gate_elements_detected = {}
    for detection in detections:
        if torch.cuda.is_available(): boxes = detection.boxes.cpu().numpy()
        else: boxes = detection.boxes.numpy()
        for box in boxes:
            bbox = list(box.xywh[0])
            x_coord = bbox[0]
            conf = float(list(box.conf)[0])
            cls_id = int(list(box.cls)[0])
            global_class_name = class_names[1][cls_id]
            if (global_class_name in ["Earth Symbol", "Abydos Symbol", "Gate"]) and conf > min_prediction_confidence:
                gate_elements_detected[cls_id] = 999999 if global_class_name == "Gate" else x_coord

    if "Earth Symbol" not in gate_elements_detected.keys(): return None
    elif "Abydos Symbol" not in gate_elements_detected.keys(): return None
    elif "Gate" not in gate_elements_detected.keys(): return None

    min_key = int(min(gate_elements_detected, key=gate_elements_detected.get))

    if min_key == "Earth Symbol": return 1.0
    else: return 0.0

def analyzeBuoy(detections, target_symbol):
    symbol_info = []
    buoy_bbox = None
    mid_x_buoy = 0
    mid_y_buoy = 0
    position = []
    for detection in detections:
        if torch.cuda.is_available(): boxes = detection.boxes.cpu().numpy()
        else: boxes = detection.boxes.numpy()
        for box in boxes:
            bbox = list(box.xywh[0])
            conf = float(list(box.conf)[0])
            cls_id = int(list(box.cls)[0])
            global_class_name = class_names[1][cls_id]
            if (global_class_name == "Buoy"):
                buoy_bbox = bbox
                mid_x_buoy = buoy_bbox[0]
                mid_y_buoy = buoy_bbox[1]
            elif (((global_class_name == "Earth Symbol" and target_symbol == "Earth Symbol") or (global_class_name == "Abydos Symbol" and target_symbol == "Abydos Symbol")) and conf > min_prediction_confidence):
                symbol_info.append(bbox)

    for bbox in symbol_info:
        if (box[0]<mid_x_buoy and box[1]<mid_y_buoy):
            position.append(1)
        elif (box[0]>mid_x_buoy and box[1]<mid_y_buoy):
            position.append(2)
        elif (box[0]<mid_x_buoy and box[1]>mid_y_buoy):
            position.append(3)
        elif (box[0]>mid_x_buoy and box[1]>mid_y_buoy):
            position.append(4)

    if buoy_bbox is None: return []
    else:
        position.sort()
        string_positions = [str(current_integer) for current_integer in position]
        string_value = "".join(string_positions)
        number = int(string_value)
        
    return number


#selects highest confidence detection from duplicates and ignores objects with no position measurement
def cleanDetections(labels, objs_x, objs_y, objs_z, objs_theta_z, extra_fields, confidences):
    label_counts = {}
    selected_detections = []

    for i in range(len(labels)):
        if None in [objs_x[i], objs_y[i], objs_z[i]]: continue
        if label_counts.get(labels[i], 0) >= max_counts_per_label[labels[i]]:
            candidate_obj_conf = confidences[i]
            min_conf_i = min(selected_detections, key=lambda x : confidences[x])
            if confidences[min_conf_i] < candidate_obj_conf:
                selected_detections.remove(min_conf_i)
                selected_detections.append(i)
        else:
            label_counts[labels[i]] = label_counts.get(labels[i], 0) + 1
            selected_detections.append(i)

    selected_labels = [labels[si] for si in selected_detections]
    selected_objs_x = [objs_x[si] for si in selected_detections]
    selected_objs_y = [objs_y[si] for si in selected_detections]
    selected_objs_z = [objs_z[si] for si in selected_detections]
    selected_objs_theta_z = [objs_theta_z[si] for si in selected_detections]
    selected_extra_fields = [extra_fields[si] for si in selected_detections]

    return selected_labels, selected_objs_x, selected_objs_y, selected_objs_z, selected_objs_theta_z, selected_extra_fields



############## ROSPY INSTANTIATIONS ##############
rospy.init_node('object_detection') #,log_level=rospy.DEBUG
#one publisher per camera
cropped_img_pubs = [
    rospy.Publisher('vision/down_cam/cropped', Image, queue_size=1),
    rospy.Publisher('vision/front_cam/cropped', Image, queue_size=1)
]
visualisation_pubs = [
    rospy.Publisher('vision/down_cam/detection', Image, queue_size=1),
    rospy.Publisher('vision/front_cam/detection', Image, queue_size=1)
    ]
pub = rospy.Publisher('vision/viewframe_detection', ObjectDetectionFrame, queue_size=1)

bridge = CvBridge()
states = (State(False), State(True))


############## PARAMETERS ##############
min_prediction_confidence = 0.4
max_dist_to_measure = 10

# [COMP] MAKE SURE THESE DIMENSIONS ARE APPROPRIATE!
pool_depth = -4.9

gate_height = 1.524
buoy_height = 1.2
octagon_table_height = 1.5 # 0.9m - 1.5m
lane_marker_height = 0.4
symbol_height = 0.3

lane_marker_top_z = -3.7 #pool_depth + lane_marker_height
octagon_table_top_z = pool_depth + octagon_table_height
gate_middle_z = 0 - (gate_height / 2)
buoy_middle_z = pool_depth + buoy_height/2 + 1.2 # 0.6m - 1.2m GUESS FROM QUALI -> hit it at -2

HEADING_COLOR = (255, 0, 0) # Blue
BOX_COLOR = (255, 255, 255) # White
TEXT_COLOR = (0, 0, 0) # Black
# [COMP] ensure FOV is correct
down_cam_hfov = 50 #set to 220 when not in sim!
down_cam_vfov = 28 #set to 165.26 when not in sim!
down_cam_yaw_offset = 0
front_cam_hfov = 78.44
front_cam_vfov = 44.12

detect_every = 5  #run the model every _ frames received (to not eat up too much RAM)
#only report predictions with confidence at least 40%


############## MODEL INSTANTIATION + PARAMETERS ##############
pwd = os.path.realpath(os.path.dirname(__file__))
# [COMP] make sure correct model is loaded
# down_cam_model_filename = pwd + "/models/down_cam_model.pt"
# front_cam_model_filename = pwd + "/models/front_cam_model.pt"
down_cam_model_filename = pwd + "/models/down_cam_model_sim.pt"
front_cam_model_filename = pwd + "/models/front_cam_sim.pt"
model = [
    YOLO(down_cam_model_filename),
    # YOLO(front_cam_model_filename)
    ]
for m in model:
    if torch.cuda.is_available(): m.to(torch.device('cuda'))
    else: print("WARN: CUDA is not available! Running on CPU")

# [COMP] update with class values for model which is trained on-site at comp
class_names = [ #one array per camera, name index should be class id
    ["Lane Marker", "Octagon Table"],
    ["Abydos Symbol", "Buoy", "Earth Symbol", "Gate", "Lane Marker", "Octagon", "Octagon Table"],
    ]
max_counts_per_label = {"Abydos Symbol":2, "Buoy":1, "Earth Symbol":2, "Gate":1, "Lane Marker":1, "Octagon":1, "Octagon Table":1}

if torch.cuda.is_available(): device=0
else: device = 'cpu'
#count for number of images received per camera
i = [
    0,
    0
    ]
