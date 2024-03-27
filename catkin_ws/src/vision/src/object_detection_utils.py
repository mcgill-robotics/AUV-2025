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
from point_cloud import get_xyz_image
from tf import transformations
from sklearn.linear_model import RANSACRegressor
from sklearn.cluster import DBSCAN

class State:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
        self.paused = False
        self.q_auv = None
        self.theta_z= None
        self.point_cloud = None
        self.depth = None
        self.bgr_img = None
        self.width = None
        self.height = None
        self.x_over_z_map = None
        self.y_over_z_map = None
        self.x_while_paused = None
        self.y_while_paused = None
        self.z_while_paused = None
        self.theta_z_while_paused = None
        self.q_auv_while_paused = None
        self.point_cloud_while_paused = None

        self.x_pos_sub = rospy.Subscriber('/state/x', Float64, self.updateX)
        self.y_pos_sub = rospy.Subscriber('/state/y', Float64, self.updateY)
        self.z_pos_sub = rospy.Subscriber('/state/z', Float64, self.updateZ)
        self.pose_sub = rospy.Subscriber('/state/pose', Pose, self.updatePose)
        self.theta_z_sub = rospy.Subscriber('/state/theta/z', Float64, self.updateThetaZ)

        # Update the point cloud whenever the current image is updated
        self.camera_info_sub = rospy.Subscriber('/vision/front_cam/camera_info', CameraInfo, self.updateCameraInfo)
        self.depth_sub = rospy.Subscriber('/vision/front_cam/aligned_depth_to_color/image_raw', Image, self.updateDepth)

    def updateX(self, msg):
        if self.paused: self.x_while_paused = float(msg.data)
        self.x = float(msg.data)
    def updateY(self, msg):
        if self.paused: self.y_while_paused = float(msg.data)
        self.y = float(msg.data)
    def updateZ(self, msg):
        if self.paused: self.z_while_paused = float(msg.data)
        self.z = float(msg.data)
    def updateThetaZ(self, msg):
        if self.paused: self.theta_z_while_paused = float(msg.data)
        self.theta_z = float(msg.data)
    def updatePose(self,msg):
        if self.paused: self.q_auv_while_paused = np.quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        self.q_auv = np.quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
    def updatePointCloud(self):
        if self.paused: self.point_cloud_while_paused = np.copy(get_xyz_image(self.depth, self.width, self.height, self.x_over_z_map, self.y_over_z_map))
        self.point_cloud = np.copy(get_xyz_image(self.depth, self.width, self.height, self.x_over_z_map, self.y_over_z_map))
    def cleanPointCloud(self, point_cloud, bgr):
        # Find the closest point to the camera
        initial_point_cloud_shape = point_cloud.shape
        point_cloud = point_cloud.reshape(-1,3)
        point_cloud[point_cloud[:,0] < 0.5] = 10000 # ignore depth values which are less than 0.5m
        closest_point_index = np.argmin(point_cloud[:,0])

        eps = 0.5  # Maximum distance between two samples for them to be considered as in the same neighborhood
        min_samples = 10  # The number of samples in a neighborhood for a point to be considered as a core point

        # Perform DBSCAN clustering
        dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        labels = dbscan.fit_predict(point_cloud)

        # Extract the cluster containing the seed point
        closest_object_label = labels[closest_point_index]
        object_mask = (labels == closest_object_label)
        point_cloud[~object_mask] = np.array([np.nan]*3)
        
        point_cloud = point_cloud.reshape(initial_point_cloud_shape)
        object_mask = object_mask.reshape(initial_point_cloud_shape[0:2])
        
        # for debugging [ONLY UNCOMMENT IF NEEDED]
        # debug_img = np.uint8(np.zeros((point_cloud.shape)))
        # debug_img[object_mask] = np.array([0,0,255])
        # rospy.Publisher('/vision/debug/point_cloud_clean', Image).publish(bridge.cv2_to_imgmsg(debug_img, "bgr8"))
        
        return point_cloud

    def getPointCloud(self, bbox=None):
        if bbox is None: 
        # bbox is bounding box: surrounds bounds an object or a specific area of interest in a robot's perception system
            return self.cleanPointCloud(np.copy(self.point_cloud), np.copy(self.bgr_img))
        else:
            return self.cleanPointCloud(cropToBbox(self.point_cloud, bbox, copy=True), cropToBbox(self.bgr_img, bbox, copy=True))
    def updateDepth(self, msg):
        temp = bridge.imgmsg_to_cv2(msg)
        self.depth = temp/depth_scale_factor
        self.updatePointCloud()
    def updateCameraInfo(self, msg):
        fx = msg.K[0]
        fy = msg.K[4]
        cx = msg.K[2]
        cy = msg.K[5]
        
        self.width = msg.width
        self.height = msg.height

        u_map = np.tile(np.arange(self.width),(self.height,1)) +1
        v_map = np.tile(np.arange(self.height),(self.width,1)).T +1

        self.x_over_z_map = (cx - u_map) / fx
        self.y_over_z_map = (cy - v_map) / fy
        if self.depth is not None:
            self.updatePointCloud()
    def pause(self):
        self.paused = True
    def resume(self):
        if self.x_while_paused is not None: self.x = self.x_while_paused
        if self.y_while_paused is not None: self.y = self.y_while_paused
        if self.z_while_paused is not None: self.z = self.z_while_paused
        if self.theta_z_while_paused is not None: self.theta_z = self.theta_z_while_paused
        if self.q_auv_while_paused is not None: self.q_auv = self.q_auv_while_paused
        if self.point_cloud_while_paused is not None: self.point_cloud = self.point_cloud_while_paused
        
        self.x_while_paused = None
        self.y_while_paused = None
        self.z_while_paused = None
        self.theta_z_while_paused = None
        self.q_auv_while_paused = None
        self.point_cloud_while_paused = None
        
        self.paused = False

# bbox is an array of 4 elements
# given an image, class name, and a bounding box, draws the bounding box rectangle and label name onto the image
def visualizeBbox(img, bbox, class_name, thickness=2, fontSize=0.5):
    #get xmin, xmax, ymin, ymax from bbox 
    x_center, y_center, w, h = bbox
    x_min = x_center - w/2
    y_min = y_center - h/2
    x_min, x_max, y_min, y_max = int(x_min), int(x_min + w), int(y_min), int(y_min + h)
    # rectangle draws bounding box on image
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
    return img # returns inputted image with label

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

def findIntersection(starting_point, vector, plane_z_pos):
    """
    Given a vector and a z position, returns the point 
    where the vector intersects the plane defined by the z position
    If they dont intersect, returns None
    """
    if vector[2] == 0: return None
    
    z_diff = plane_z_pos - starting_point[2]
    
    scaling_factor = z_diff / vector[2]
    if scaling_factor < 0: return None
    
    return starting_point + np.array(vector) * scaling_factor

def getObjectPositionDownCam(pixel_x, pixel_y, img_height, img_width, z_pos):
    """
    Given the pixel locations and height and width

    Parameters:
        pixel_x: x coordinate of the object in the image
        pixel_y: y coordinate of the object in the image
        img_height: height of the image in pixels
        img_width: width of the image in pixels
        z_pos: z position of the object

    Returns:
        x, y, z position in 3D space (not relative to the AUV)
    """
    #first calculate the relative offset of the object from the center of the image (i.e. map pixel coordinates to values from -0.5 to 0.5)
    x_center_offset = ((img_width/2) - pixel_x) / img_width #-0.5 to 0.5
    y_center_offset = (pixel_y - (img_height/2)) / img_height #negated since y goes from top to bottom
    #use offset within image and total FOV of camera to find an angle offset from the angle the camera is facing
    #assuming FOV increases linearly with distance from center pixel
    roll_angle_offset = down_cam_hfov*x_center_offset
    pitch_angle_offset = down_cam_vfov*y_center_offset
    
    x_pos_offset = -math.tan(math.radians(pitch_angle_offset))
    y_pos_offset = math.tan(math.radians(roll_angle_offset))
    
    local_direction_to_object = np.array([x_pos_offset,y_pos_offset,-1])
    
    rotation_offset = transformations.quaternion_from_euler(0, 0, down_cam_yaw_offset)
    rotation = states[0].q_auv * np.quaternion(rotation_offset[3], rotation_offset[0], rotation_offset[1], rotation_offset[2])
    global_direction_to_object = quaternion.rotate_vectors(rotation, local_direction_to_object)

    # solve for point that is defined by the intersection of the direction to the object and it's z position
    global_down_cam_offset = quaternion.rotate_vectors(states[0].q_auv, np.array([down_cam_x_offset, down_cam_y_offset, down_cam_z_offset]))
    down_cam_pos = np.array([states[0].x, states[0].y, states[0].z]) + global_down_cam_offset
    obj_pos = findIntersection(down_cam_pos, global_direction_to_object, z_pos)
    
    if obj_pos is None or np.linalg.norm(obj_pos - np.array([states[0].x, states[0].y, states[0].z])) > max_dist_to_measure: return None, None, None
    x = obj_pos[0]
    y = obj_pos[1]
    z = z_pos
    return x, y, z

# given a bounding box, tells you where the main object in the bounding box is in 3D space (world space)
# assumes cleaning was correct
def getObjectPositionFrontCam(bbox):
    point_cloud = states[1].getPointCloud(bbox)
    min_lx = np.nanmin(point_cloud[:,:,0].flatten())
    min_ly = np.nanmin(point_cloud[:,:,1].flatten())
    min_lz = np.nanmin(point_cloud[:,:,2].flatten())
    max_lx = np.nanmax(point_cloud[:,:,0].flatten())
    max_ly = np.nanmax(point_cloud[:,:,1].flatten())
    max_lz = np.nanmax(point_cloud[:,:,2].flatten())
    
    lx = (max_lx + min_lx) / 2
    ly = (max_ly + min_ly) / 2
    lz = (max_lz + min_lz) / 2
    
    global_obj_pos_offset = quaternion.rotate_vectors(states[1].q_auv, np.array([lx,ly,lz]))
    
    # Get the best estimate of the mean
    x,y,z = global_obj_pos_offset + np.array([states[1].x, states[1].y, states[1].z])
    return x, y, z

# tells you how the object is oriented in space (Z axis)
def measureAngle(bbox):
    """
    Given a bounding box, returns the angle of the object in degrees (only for front cam)
    """
    point_cloud = states[1].getPointCloud(bbox)

    point_cloud_x = point_cloud[:,:,0].flatten() # collect x, ignore z positions of points
    point_cloud_y = point_cloud[:,:,1].flatten() # collect y, ignore z position of points

    nan_indices = np.isnan(point_cloud_x) | np.isnan(point_cloud_y) # find indices to remove (NaNs)

    filtered_point_cloud_x = point_cloud_x[~nan_indices] # filter point cloud x so it has no NaNs
    filtered_point_cloud_y = point_cloud_y[~nan_indices] # filter point cloud y so it has no NaNs

    # fit a line to the point cloud X/Y with RANSAC (removes outliers)
    ransac = RANSACRegressor()
    ransac.fit(-1 * filtered_point_cloud_y.reshape(-1,1), filtered_point_cloud_x)
    slope = ransac.estimator_.coef_[0]

    angle = math.degrees(math.atan(slope)) # Calculate the angle of the fitted line
    
    return angle + states[1].theta_z

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

        if torch.cuda.is_available(): 
            boxes = detection.boxes.cpu().numpy()
        else: 
            boxes = detection.boxes.numpy()

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

    if buoy_was_detected: 
        return symbols

    else: 
        return []

# lots of noise in pool, the idea is for example if the down cam has two detections, it will remove the least confident one
# selects highest confidence detection from duplicates and ignores objects with no position measurement
def cleanDetections(detectionFrameArray):
    label_counts = {}
    selected_detections = []

    for i in range(len(detectionFrameArray)):
        obj = detectionFrameArray[i]
        if None in [obj.x, obj.y, obj.z]: continue
        if label_counts.get(obj.label, 0) >= max_counts_per_label[obj.label]:
            candidate_obj_conf = obj.confidence
            min_conf_i = min(selected_detections, key=lambda x : detectionFrameArray[x].confidence)
            if detectionFrameArray[min_conf_i].confidence < candidate_obj_conf:
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

HEADING_COLOR = (255, 0, 0) # Blue
BOX_COLOR = (255, 255, 255) # White
TEXT_COLOR = (0, 0, 0) # Black

# [COMP] MAKE SURE THESE DIMENSIONS ARE APPROPRIATE!
pool_depth = -5
octagon_table_height = 1.25 # 0.9m - 1.5m
lane_marker_height = 0.4
lane_marker_top_z = pool_depth + lane_marker_height
octagon_table_top_z = pool_depth + octagon_table_height
# [COMP] ensure FOV is correct
down_cam_hfov = 129.4904
down_cam_vfov = 100
down_cam_x_offset = rospy.get_param("down_cam_x_offset", 0)
down_cam_y_offset = rospy.get_param("down_cam_y_offset", 0)
down_cam_z_offset = rospy.get_param("down_cam_z_offset", 0)
down_cam_yaw_offset = rospy.get_param("down_cam_yaw_offset", 0)

detect_every = 5  #run the model every _ frames received (to not eat up too much RAM)

############## MODEL INSTANTIATION + PARAMETERS ##############
pwd = os.path.realpath(os.path.dirname(__file__))

sim = rospy.get_param("sim", True)

down_cam_model_filename = ""
front_cam_model_filename = ""

# Select the proper models & depth_scale_factor based on the sim argument
if sim:
    down_cam_model_filename = pwd + "/models/down_cam_model_sim.pt"
    front_cam_model_filename = pwd + "/models/front_cam_model.pt"
    depth_scale_factor = 1
else:
    down_cam_model_filename = pwd + "/models/down_cam_model_sim.pt"
    front_cam_model_filename = pwd + "/models/front_cam_model.pt"
    depth_scale_factor = 1000

model = [
    YOLO(down_cam_model_filename),
    YOLO(front_cam_model_filename)
    ]

if not torch.cuda.is_available(): rospy.logwarn("CUDA is not available! YOLO inference will run on CPU.")

for m in model:
    if torch.cuda.is_available(): m.to(torch.device('cuda'))

# [COMP] update with class values for model which is trained on-site at comp
class_names = [ #one array per camera, name index should be class id
    ["Lane Marker", "Octagon Table"],
    ["Abydos Symbol", "Buoy", "Earth Symbol", "Gate", "Lane Marker", "Octagon", "Octagon Table"],
    ]
max_counts_per_label = {"Abydos Symbol":2, "Buoy":1, "Earth Symbol":2, "Gate":1, "Lane Marker":2, "Octagon Table":1}

if torch.cuda.is_available(): device=0
else: device = 'cpu'
#count for number of images received per camera
i = [
    0,
    0
    ]
    
