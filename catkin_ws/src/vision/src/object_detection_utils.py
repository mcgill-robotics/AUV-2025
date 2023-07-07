import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import math
from std_msgs.msg import Float64
import lane_marker_measure
import torch
from geometry_msgs.msg import Pose
import quaternion

def transformLocalToGlobal(lx,ly,lz,camera_id):
    rotation = states[camera_id].q_auv
    return quaternion.rotate_vectors(rotation, np.array([lx,ly,lz]))

class State:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
        self.theta_x = None
        self.theta_y = None
        self.theta_z = None
        self.depth_map = None
        self.paused = False
        self.q_auv = None
        self.x_pos_sub = rospy.Subscriber('state_x', Float64, self.updateX)
        self.y_pos_sub = rospy.Subscriber('state_y', Float64, self.updateY)
        self.z_pos_sub = rospy.Subscriber('state_z', Float64, self.updateZ)
        self.pose_sub = rospy.Subscriber('pose', Pose, self.updatePose)
        self.theta_x_sub = rospy.Subscriber('state_theta_x', Float64, self.updateThetaX)
        self.theta_y_sub = rospy.Subscriber('state_theta_y', Float64, self.updateThetaY)
        self.theta_z_sub = rospy.Subscriber('state_theta_z', Float64, self.updateThetaZ)
        self.depth_sub = rospy.Subscriber('vision/front_cam/depth', Image, self.updateDepthMap)
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
    def updateDepthMap(self, msg):
        if self.paused: return
        self.depth_map = np.copy(bridge.imgmsg_to_cv2(msg, "passthrough"))
        self.depth_map += np.random.normal(0, 0.1, self.depth_map.shape)
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

def clean_depth_error(depth_img):
    """ set all the points with a depth error smaller than the threshold to 100 """
    depth_img[depth_img <= 0.6] = 10000
    depth_img[depth_img >= 10] = 10000
    return depth_img

def remove_background(depth_img):
    """ set all the points further than the min point by 3 meters to NaN """
    min_point = np.nanmin(depth_img)
    depth_img[depth_img > 3 + min_point] = np.NaN
    return depth_img

def gate_depth(depth_cropped):
    """ divide the picture in two (left and right) - get the mean of the 50 min value on each side -
    mean the two means """
    _, cols = depth_cropped.shape
    left_half, right_half = depth_cropped[:, :int(cols/2)], depth_cropped[:,int(cols/2):]
    left_flattened = left_half.flatten()
    left_smallest_values = np.partition(left_flattened, 50)[:50]
    right_flatten = right_half.flatten()
    right_smallest_values = np.partition(right_flatten, 50)[:50]
    return (np.nanmean(left_smallest_values) + np.nanmean(right_smallest_values)) / 2

def buoy_depth(depth_cropped):
    """ get the middle half of the image and take the mean """
    rows, cols = depth_cropped.shape
    return np.nanmean(depth_cropped[int(rows/4):int(3*rows/4), int(cols/4):int(3*cols/4)])

def lane_marker_depth(depth_cropped):
    return np.nanmean(depth_cropped)

def octagon_table_depth(depth_cropped):
    """ get the 50 smallest values, take the mean and add 0.6096m to it. The octagon table has width and depth = 2 * 0.6096; 
        by adding 0.6096 to the mean of the 50 smallest values, we get the distance from the camera to the center of the octagon table """
    rows, cols = depth_cropped.shape
    center_depth = depth_cropped[int(rows/4):int(3*rows/4), int(cols/4):int(3*cols/4)]
    smallest_values = np.partition(center_depth.flatten(), 50)[:50]
    smallest_mean = np.nanmean(smallest_values)
    return smallest_mean + 0.6096

def object_depth(depth_cropped, label):
    depth_cropped = clean_depth_error(depth_cropped)
    depth_cropped = remove_background(depth_cropped)
    dist = 0
    if label == 0:
        dist = lane_marker_depth(depth_cropped)
    elif label == 1:
        dist = gate_depth(depth_cropped)
    elif label == 2:
        dist = buoy_depth(depth_cropped)
    elif label == 3:
        dist = octagon_table_depth(depth_cropped)

    return dist

def eulerToVectorDownCam(x_deg, y_deg):
    x_rad = math.radians(x_deg)
    y_rad = math.radians(y_deg)
    x = -math.tan(y_rad)
    y = math.tan(x_rad)
    # we want sqrt(x**2 + y**2 + z**2) == 1
    #   z**2 == 1 - x**2 + y**2
    #   z == -1 * sqrt(1 - x**2 + y**2)
    try:
        z = -1 * abs(math.sqrt(1 - (x ** 2 + y ** 2)))
        vec = np.array([x,y,z])
    except ValueError:
        vec = np.array([x,y,0])
        vec = vec / np.linalg.norm(vec)
    return vec

def eulerToVectorFrontCam(x_deg, y_deg):
    x_rad = math.radians(x_deg)
    y_rad = math.radians(y_deg)
    y = math.tan(x_rad)
    z = -math.tan(y_rad)
    # we want sqrt(x**2 + y**2 + z**2) == 1
    #   x**2 == 1 - y**2 + z**2
    #   x == sqrt(1 - y**2 + z**2)
    try:
        x = abs(math.sqrt(1 - (y ** 2 + z ** 2)))
        vec = np.array([x,y,z])
    except ValueError:
        vec = np.array([0,y,z])
        vec = vec / np.linalg.norm(vec)
    return vec

def find_intersection(vector, plane_z_pos):
    if vector[2] == 0: return None

    scaling_factor = plane_z_pos / vector[2]
    if scaling_factor < 0: return None

    return np.array(vector) * scaling_factor

def getObjectPosition(pixel_x, pixel_y, img_height, img_width, dist_from_camera=None, z_pos=None):
    if dist_from_camera is not None: # ASSUMES FRONT CAMERA
        #first calculate the relative offset of the object from the center of the image (i.e. map pixel coordinates to values from -0.5 to 0.5)
        x_center_offset = ((img_width/2) - pixel_x) / img_width #-0.5 to 0.5
        y_center_offset = (pixel_y - (img_height/2)) / img_height #negated since y goes from top to bottom
        #use offset within image and total FOV of camera to find an angle offset from the angle the camera is facing
        #assuming FOV increases linearly with distance from center pixel
        yaw_angle_offset = front_cam_hfov*x_center_offset
        pitch_angle_offset = front_cam_vfov*y_center_offset
        
        direction_to_object = eulerToVectorFrontCam(yaw_angle_offset, pitch_angle_offset)
        vector_to_object = direction_to_object * dist_from_camera
        
        #convert local offsets to global offsets using tf transform library
        x,y,z = transformLocalToGlobal(vector_to_object[0], vector_to_object[1], vector_to_object[2], 1)
        return x,y,z
    elif z_pos is not None: # ASSUMES DOWN CAMERA
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
    else:
        print("! ERROR: Not enough information to calculate a position! Require at least a known z position or a distance from the camera.")
        return None, None, None

def measureBuoyAngle(depth_img, buoy_width, bbox_coordinates):
    depth_img = clean_depth_error(depth_img)
    depth_img = remove_background(depth_img)
    depth_cropped = cropToBbox(depth_img, bbox_coordinates)
    _, width = depth_cropped.shape
    left_half, right_half = depth_cropped[:, :int(width/2)], depth_cropped[:, int(width/2):]
    avg_left_depth = np.nanmin(left_half)
    avg_right_depth = np.nanmin(right_half)

    left_pole_angle = (180 * math.acos((buoy_width**2 + avg_left_depth**2 - avg_right_depth**2)/(2*buoy_width*avg_left_depth)) / math.pi) - 90
    buoy_pixel_x_left = bbox_coordinates[0] - bbox_coordinates[2]/2
    x_center_offset = ((depth_img.shape[1]/2) - buoy_pixel_x_left) / depth_img.shape[1] #-0.5 to 0.5
    theta_x = front_cam_hfov * x_center_offset
    buoy_angle = states[1].theta_z + left_pole_angle + theta_x

    return buoy_angle
    
def measureGateAngle(depth_img, gate_width, bbox_coordinates): # ELIE
    depth_img = clean_depth_error(depth_img)
    depth_img = remove_background(depth_img)
    depth_cropped = cropToBbox(depth_img, bbox_coordinates)
    _, width = depth_cropped.shape
    left_half, right_half = depth_cropped[:, :int(width/2)], depth_cropped[:, int(width/2):]
    avg_left_depth = np.nanmin(left_half)
    avg_right_depth = np.nanmin(right_half)
    left_pole_angle = (180 * math.acos((gate_width**2 + avg_left_depth**2 - avg_right_depth**2)/(2*gate_width*avg_left_depth)) / math.pi) - 90
    # auv_angle = math.acos((avg_left_depth**2 +avg_right_depth**2 - gate_width**2)/(2*avg_left_depth*avg_right_depth))
    # right_pole_angle = 180 - auv_angle - left_pole_angle
    gate_pixel_x_left = bbox_coordinates[0] - bbox_coordinates[2]/2
    x_center_offset = ((depth_img.shape[1]/2) - gate_pixel_x_left) / depth_img.shape[1] #-0.5 to 0.5
    theta_x = front_cam_hfov*x_center_offset
    gate_angle = states[1].theta_z + left_pole_angle + theta_x
    return gate_angle

def analyzeGate(detections, min_confidence, earth_class_id, abydos_class_id, gate_class_id): # AYOUB
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
            if (cls_id == earth_class_id or cls_id == abydos_class_id or cls_id == gate_class_id) and conf > min_confidence:
                gate_elements_detected[cls_id] = 999999 if cls_id == gate_class_id else x_coord

    if earth_class_id not in gate_elements_detected.keys(): return None
    elif abydos_class_id not in gate_elements_detected.keys(): return None
    elif gate_class_id not in gate_elements_detected.keys(): return None

    min_key = int(min(gate_elements_detected, key=gate_elements_detected.get))

    if min_key == earth_class_id: return 1.0
    else: return 0.0
    

def analyzeBuoy(detections, min_confidence, earth_class_id, abydos_class_id, buoy_class_id, depth_map):
    return []

def cleanDetections(labels, objs_x, objs_y, objs_z, objs_theta_z, extra_fields, confidences, max_counts_per_label):
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

HEADING_COLOR = (255, 0, 0) # Blue
BOX_COLOR = (255, 255, 255) # White
TEXT_COLOR = (0, 0, 0) # Black
down_cam_hfov = 87 #set to 220 when not in sim!
down_cam_vfov = 65 #set to 165.26 when not in sim!
front_cam_hfov = 87
front_cam_vfov = 58

max_dist_to_measure = 10

bridge = CvBridge()
states = (State(), State())
