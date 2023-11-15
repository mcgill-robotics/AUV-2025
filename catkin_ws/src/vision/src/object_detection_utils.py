import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from ultralytics import YOLO
from auv_msgs.msg import VisionObjectArray
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
    def __init__(self):
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
        self.point_cloud_sub = rospy.Subscriber('vision/front_cam/point_cloud_raw', Image, self.updatePointCloud, queue_size=1)
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
    def updatePointCloud(self, msg):
        if self.paused: return
        self.point_cloud = np.copy(bridge.imgmsg_to_cv2(msg, "passthrough"))
    def cleanPointCloud(self, point_cloud):
        #APPLY MEDIAN BLUR FILTER TO REMOVE SALT AND PEPPER NOISE
        median_blur_size = 5
        point_cloud = cv2.medianBlur(point_cloud, median_blur_size)
        #REMOVE BACKGROUND (PIXELS TOO FAR AWAY FROM CLOSEST PIXEL)
        closest_x_point = np.nanmin(point_cloud[:, :, 0])
        far_mask = point_cloud[:, :, 0] > closest_x_point + 2 #set to 2 instead of 3 since the gate will never be perfectly orthogonal to the camera
        point_cloud[far_mask] = np.array([np.nan, np.nan, np.nan])
        return point_cloud
    def getPointCloud(self, bbox=None):
        if bbox is None:
            return self.cleanPointCloud(self.point_cloud)
        else:
            return self.cleanPointCloud(cropToBbox(self.point_cloud, bbox, copy=True))
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

def eulerToVectorDownCam(x_deg, y_deg):
    x_rad = math.radians(x_deg)
    y_rad = math.radians(y_deg)
    x = -math.tan(y_rad)
    y = math.tan(x_rad)
    vec = np.array([x,y,-1])
    vec = vec / np.linalg.norm(vec)
    return vec

def findIntersection(vector, plane_z_pos):
    if vector[2] == 0: return None
    scaling_factor = plane_z_pos / vector[2]
    if scaling_factor < 0: return None
    return np.array(vector) * scaling_factor

def getObjectPositionDownCam(pixel_x, pixel_y, img_height, img_width, z_pos):
    #first calculate the relative offset of the object from the center of the image (i.e. map pixel coordinates to values from -0.5 to 0.5)
    x_center_offset = ((img_width/2) - pixel_x) / img_width #-0.5 to 0.5
    y_center_offset = (pixel_y - (img_height/2)) / img_height #negated since y goes from top to bottom
    #use offset within image and total FOV of camera to find an angle offset from the angle the camera is facing
    #assuming FOV increases linearly with distance from center pixel
    roll_angle_offset = down_cam_hfov*x_center_offset
    pitch_angle_offset = down_cam_vfov*y_center_offset

    local_direction_to_object = eulerToVectorDownCam(roll_angle_offset, pitch_angle_offset)
    global_direction_to_object = transformLocalToGlobal(local_direction_to_object[0], local_direction_to_object[1], local_direction_to_object[2], 0, yaw_offset=down_cam_yaw_offset)

    # solve for point that is defined by the intersection of the direction to the object and it's z position
    obj_pos = findIntersection(global_direction_to_object, z_pos)
    if obj_pos is None or np.linalg.norm(obj_pos - np.array([states[0].x, states[0].y, states[0].z])) > max_dist_to_measure: return None, None, None
    x = obj_pos[0]
    y = obj_pos[1]
    z = z_pos
    return x, y, z

def getObjectPositionFrontCam(bbox):
    point_cloud = states[1].getPointCloud(bbox)
    lx = np.nanmean(point_cloud[:,:,0])
    ly = np.nanmean(point_cloud[:,:,1])
    lz = np.nanmean(point_cloud[:,:,2])
    x,y,z = transformLocalToGlobal(lx,ly,lz,camera_id=1)
    return x, y, z

def measureAngle(bbox):
    point_cloud = states[1].getPointCloud(bbox) # ignore z position of points
    left_point_cloud = point_cloud[:, :int(point_cloud.shape[1]/2)]
    right_point_cloud = point_cloud[:, int(point_cloud.shape[1]/2):]
    #avg left points together and right points together so we get two (x,y) points
    local_left_avg_point = np.nanmean(left_point_cloud, axis=(0,1))
    local_right_avg_point = np.nanmean(right_point_cloud, axis=(0,1))
    #measure angle of vector defined by averaged left/right points
    left_x,left_y,left_z = transformLocalToGlobal(local_left_avg_point[0], local_left_avg_point[1], local_left_avg_point[2], camera_id=1)
    right_x,right_y,right_z = transformLocalToGlobal(local_right_avg_point[0], local_right_avg_point[1], local_right_avg_point[2], camera_id=1)
    left_avg_point = np.array([left_x,left_y,left_z])
    right_avg_point = np.array([right_x,right_y,right_z])
    zero_angle_vector = np.array([0,-1])
    arg_vector = right_avg_point - left_avg_point
    magnitude_arg_vector = np.linalg.norm(arg_vector)
    dot_product = np.dot(zero_angle_vector, arg_vector)
    return math.acos(dot_product / magnitude_arg_vector) * 180 / math.pi

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

def analyzeBuoy(detections):
    symbols = []
    buoy_was_detected = False
    for detection in detections:
        if torch.cuda.is_available(): boxes = detection.boxes.cpu().numpy()
        else: boxes = detection.boxes.numpy()
        for box in boxes:
            bbox = list(box.xywh[0])
            conf = float(list(box.conf)[0])
            cls_id = int(list(box.cls)[0])
            global_class_name = class_names[1][cls_id]
            if (global_class_name == "Buoy"):
                buoy_was_detected = True
            elif (global_class_name in ["Earth Symbol", "Abydos Symbol"]) and conf > min_prediction_confidence:
                x,y,z = getObjectPositionFrontCam(bbox)
                symbols.append([global_class_name, x, y, z, conf])

    if buoy_was_detected: return symbols
    else: return []


#selects highest confidence detection from duplicates and ignores objects with no position measurement
def cleanDetections(detectionFrameArray, confidences):
    label_counts = {}
    selected_detections = []

    for i in range(len(detectionFrameArray)):
        obj = detectionFrameArray[i]
        if None in [obj.x, obj.y, obj.z]: continue
        if label_counts.get(obj.label, 0) >= max_counts_per_label[obj.label]:
            candidate_obj_conf = confidences[i]
            min_conf_i = min(selected_detections, key=lambda x : confidences[x])
            if confidences[min_conf_i] < candidate_obj_conf:
                selected_detections.remove(min_conf_i)
                selected_detections.append(i)
        else:
            label_counts[obj.label] = label_counts.get(obj.label, 0) + 1
            selected_detections.append(i)


    return [detectionFrameArray[i] for i in selected_detections]



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
pub = rospy.Publisher('vision/viewframe_detection', VisionObjectArray, queue_size=1)

bridge = CvBridge()
states = (State(), State())


############## PARAMETERS ##############
min_prediction_confidence = 0.4
max_dist_to_measure = 10

# [COMP] MAKE SURE THESE DIMENSIONS ARE APPROPRIATE!
pool_depth = -4.9
octagon_table_height = 1.5 # 0.9m - 1.5m
lane_marker_height = 0.4
lane_marker_top_z = -3.7 #pool_depth + lane_marker_height
octagon_table_top_z = pool_depth + octagon_table_height

HEADING_COLOR = (255, 0, 0) # Blue
BOX_COLOR = (255, 255, 255) # White
TEXT_COLOR = (0, 0, 0) # Black
# [COMP] ensure FOV is correct
down_cam_hfov = 50 #set to 220 when not in sim!
down_cam_vfov = 28 #set to 165.26 when not in sim!
down_cam_yaw_offset = 0

detect_every = 5  #run the model every _ frames received (to not eat up too much RAM)
#only report predictions with confidence at least 40%


############## MODEL INSTANTIATION + PARAMETERS ##############
pwd = os.path.realpath(os.path.dirname(__file__))

sim = rospy.get_param("sim")

down_cam_model_filename = ""
front_cam_model_filename = ""

# Select the proper models based on the sim argument
if sim:
    down_cam_model_filename = pwd + "/models/down_cam_model_sim.pt"
    front_cam_model_filename = pwd + "/models/front_cam_sim.pt"
else:
    down_cam_model_filename = pwd + "/models/down_cam_model.pt"
    front_cam_model_filename = pwd + "/models/front_cam_model.pt"

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
max_counts_per_label = {"Abydos Symbol":2, "Buoy":1, "Earth Symbol":2, "Gate":1, "Lane Marker":1, "Octagon Table":1}

if torch.cuda.is_available(): device=0
else: device = 'cpu'
#count for number of images received per camera
i = [
    0,
    0
    ]
