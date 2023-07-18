import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
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
from sensor_msgs import point_cloud2
class State:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
        self.theta_x = None
        self.theta_y = None
        self.theta_z = None
        self.point_cloud = None
        self.paused = False
        self.q_auv = None
        self.x_pos_sub = rospy.Subscriber('state_x', Float64, self.updateX)
        self.y_pos_sub = rospy.Subscriber('state_y', Float64, self.updateY)
        self.z_pos_sub = rospy.Subscriber('state_z', Float64, self.updateZ)
        self.pose_sub = rospy.Subscriber('pose', Pose, self.updatePose)
        self.theta_x_sub = rospy.Subscriber('state_theta_x', Float64, self.updateThetaX)
        self.theta_y_sub = rospy.Subscriber('state_theta_y', Float64, self.updateThetaY)
        self.theta_z_sub = rospy.Subscriber('state_theta_z', Float64, self.updateThetaZ)
        self.point_cloud_sub = rospy.Subscriber('vision/front_cam/point_cloud', PointCloud2, self.updatePointCloud)
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
    def updatePointCloud(self, msg):
        if self.paused: return
        pc = point_cloud2.read_points_list(msg)
        pc = np.array(pc)
        self.point_cloud = pc[:,[2,0,1]]
        self.point_cloud[:,2] *= -1
        self.point_cloud = self.point_cloud.reshape(msg.height, msg.width, 3)
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

def transformLocalToGlobal(lx,ly,lz,camera_id):
    rotation = states[camera_id].q_auv
    return quaternion.rotate_vectors(rotation, np.array([lx,ly,lz]))

def eulerToVectorDownCam(x_deg, y_deg):
    x_rad = math.radians(x_deg)
    y_rad = math.radians(y_deg)
    x = -math.tan(y_rad)
    y = math.tan(x_rad)
    try:
        z = -1 * abs(math.sqrt(1 - (x ** 2 + y ** 2)))
        vec = np.array([x,y,z])
    except ValueError:
        vec = np.array([x,y,0])
        vec = vec / np.linalg.norm(vec)
    return vec

def find_intersection(vector, plane_z_pos):
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
    global_direction_to_object = transformLocalToGlobal(local_direction_to_object[0], local_direction_to_object[1], local_direction_to_object[2], 0)

    # solve for point that is defined by the intersection of the direction to the object and it's z position
    obj_pos = find_intersection(global_direction_to_object, z_pos)
    if obj_pos is None or np.linalg.norm(obj_pos - np.array([states[0].x, states[0].y, states[0].z])) > max_dist_to_measure: return None, None, None
    x = obj_pos[0]
    y = obj_pos[1]
    z = z_pos
    return x, y, z

def cleanPointCloud(point_cloud):
    #APPLY MEDIAN BLUR FILTER TO REMOVE SALT AND PEPPER NOISE
    median_blur_size = 10
    point_cloud = cv2.medianBlur(point_cloud, median_blur_size)
    #REMOVE BACKGROUND (PIXELS TOO FAR AWAY FROM CLOSEST PIXEL)
    closest_x_point = np.nanmin(point_cloud[:, :, 0])
    far_mask = point_cloud[:, :, 0] > closest_x_point + 2 #set to 2 instead of 3 since the gate will never be perfectly orthogonal to the camera
    point_cloud[far_mask] = np.array([np.nan, np.nan, np.nan])
    return point_cloud

def getObjectPositionFrontCam(bbox):
        cropped_point_cloud = cleanPointCloud(cropToBbox(states[1].point_cloud, bbox, copy=True))
        lx = np.nanmean(cropped_point_cloud[:,:,0])
        ly = np.nanmean(cropped_point_cloud[:,:,1])
        lz = np.nanmean(cropped_point_cloud[:,:,2])
        x,y,z = transformLocalToGlobal(lx,ly,lz,camera_id=1)
        return states[1].x + x, states[1].y + y, states[1].z + z
        # TODO! SEPERATE CASE FOR GATE?

def measureAngle(bbox, global_class_name):
    if global_class_name in ["Gate", "Buoy"]:
        cropped_point_cloud = cleanPointCloud(cropToBbox(states[1].point_cloud, bbox, copy=True))[:,:,0:2] # ignore z position of points
        left_point_cloud = cropped_point_cloud[:, :int(cropped_point_cloud.shape[1]/2)]
        right_point_cloud = cropped_point_cloud[:, int(cropped_point_cloud.shape[1]/2):]
        #sum left points together and right points together so we get two very large (x,y) points
        left_sum_point = np.nansum(left_point_cloud, axis=(0,1))
        right_sum_point = np.nansum(right_point_cloud, axis=(0,1))
        #measure angle of summed vector (effectively a weight average where the weight is the magnitude of the vector)
        return measureYaw(left_sum_point, right_sum_point)
    else: return None

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
                continue
            elif (global_class_name in ["Earth Symbol", "Abydos Symbol"]) and conf > min_prediction_confidence:
                x,y,z = getObjectPositionFrontCam(bbox)
                symbols.append([global_class_name, x, y, z, conf])

    if buoy_was_detected: return symbols
    else: return []

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
states = (State(), State())


############## PARAMETERS ##############
min_prediction_confidence = 0.4
max_dist_to_measure = 10

# [COMP] change pool depth and object heights to actual comp values
pool_depth = -4
lane_marker_z = pool_depth + 0.3
octagon_table_z = pool_depth + 1.2

HEADING_COLOR = (255, 0, 0) # Blue
BOX_COLOR = (255, 255, 255) # White
TEXT_COLOR = (0, 0, 0) # Black
# [COMP] ensure FOV is correct
down_cam_hfov = 87 #set to 220 when not in sim!
down_cam_vfov = 65 #set to 165.26 when not in sim!

detect_every = 5  #run the model every _ frames received (to not eat up too much RAM)
#only report predictions with confidence at least 40%


############## MODEL INSTANTIATION + PARAMETERS ##############
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