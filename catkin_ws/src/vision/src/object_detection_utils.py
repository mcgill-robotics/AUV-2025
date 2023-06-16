import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import math
from geometry_msgs.msg import Vector3, Vector3Stamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from std_msgs.msg import Header, Float64
import lane_marker_measure

def transformLocalToGlobal(lx,ly,lz):
    trans = tf_buffer.lookup_transform("world", "auv_base", rospy.Time(0))
    offset_local = Vector3(lx, ly, lz)
    tf_header.stamp = rospy.Time(0)
    offset_local_stmp = Vector3Stamped(header=tf_header, vector=offset_local)
    offset_global = tf2_geometry_msgs.do_transform_vector3(offset_local_stmp, trans)
    return float(offset_global.vector.x), float(offset_global.vector.y), float(offset_global.vector.z)

class State:
    def __init__(self):
        self.x_pos_sub = rospy.Subscriber('state_x', Float64, self.updateX)
        self.y_pos_sub = rospy.Subscriber('state_y', Float64, self.updateY)
        self.z_pos_sub = rospy.Subscriber('state_z', Float64, self.updateZ)
        self.theta_x_sub = rospy.Subscriber('state_theta_x', Float64, self.updateThetaX)
        self.theta_y_sub = rospy.Subscriber('state_theta_y', Float64, self.updateThetaY)
        self.theta_z_sub = rospy.Subscriber('state_theta_z', Float64, self.updateThetaZ)
        self.depth_sub = rospy.Subscriber('vision/front_cam/depth', Image, self.updateDepthMap)
        self.x = None
        self.y = None
        self.z = None
        self.theta_x = None
        self.theta_y = None
        self.theta_z = None
        self.depth_map = None
def updateX(self, msg):
    self.x = float(msg.data)
def updateY(self, msg):
    self.y = float(msg.data)
def updateZ(self, msg):
    self.z = float(msg.data)
def updateThetaX(self, msg):
    self.theta_x = float(msg.data)
def updateThetaY(self, msg):
    self.theta_y = float(msg.data)
def updateThetaZ(self, msg):
    self.theta_z = float(msg.data)
def updateDepthMap(self, msg):
    self.depth = bridge.imgmsg_to_cv2(msg, "passthrough")

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
        lineType=cv2.LINE_AA,
    )
    return img

#given a bounding box and image, returns the image cropped to the bounding box (to isolate detected objects)
def cropToBbox(img, bbox):
    x_center, y_center, w, h = bbox
    x_min = x_center - w/2
    y_min = y_center - h/2
    x_min, x_max, y_min, y_max = int(x_min), int(x_min + w), int(y_min), int(y_min + h)
    crop_img = img[y_min:y_max, x_min:x_max]
    return crop_img

def measureLaneMarker(img, bbox, debug_img):
    #crop image to lane marker
    cropped_img = cropToBbox(img, bbox)
    line_thickness = 2 # in pixels
    line_length = 0.25*min(bbox[2], bbox[3]) #line will be size of shortest bounding box side
    #measure headings from lane marker
    cropped_img_to_pub = bridge.cv2_to_imgmsg(cropped_img, "bgr8")
    cropped_img_pub.publish(cropped_img_to_pub)
    headings, center_point = lane_marker_measure.measure_headings(cropped_img)
    if None in (headings, center_point): return (None, None), (None, None), debug_img
    center_point_x = center_point[0] + bbox[0] - bbox[2]/2
    center_point_y = center_point[1] + bbox[1] - bbox[3]/2
    center_point = (int(center_point_x), int(center_point_y))
    for angle in headings:
        #get angle, line start and line end from heading slope
        slope = math.tan((angle/-180)*math.pi)
        #calculate line x length from total length
            #line_length = sqrt(line_x_length^2 + line_y_length^2)
            #line_length^2 = line_x_length^2 + (line_x_length*slope)^2
            #line_length^2 = line_x_length^2 * (1 + slope^2)
            #line_x_length = sqrt(line_length^2 / (1 + slope^2))
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
            lineType=cv2.LINE_AA,
        )
    cv2.circle(debug_img, center_point, radius=5, color=HEADING_COLOR, thickness=-1)
    return headings, center_point, debug_img

def gate_depth(depth_cropped):
    rows, _ = depth_cropped.shape
    left_half, right_half = depth_cropped[:rows/2, :], depth_cropped[rows/2:, :]
    left_closest = np.unravel_index(np.argmin(left_half), left_half.shape)
    right_closest = np.unravel_index(np.argmin(right_half), right_half.shape)
    return (left_closest + right_closest) / 2

def buoy_depth(depth_cropped):
    rows, cols = depth_cropped.shape
    middle_point = depth_cropped[rows/2, cols/2]
    return middle_point

def object_depth(depth_cropped, label, error_threshold=0.5):
    depth_cropped[depth_cropped <= error_threshold] = 100
    dist = 0
    if label == 2:
        dist = buoy_depth(depth_cropped)
    elif label == 1:
        dist = gate_depth(depth_cropped)
    return dist

def eulerToVector(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg)
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)

    cos_roll = math.cos(roll)
    cos_yaw = math.cos(yaw)
    cos_pitch = math.cos(pitch)
    sin_roll = math.sin(roll)
    sin_yaw = math.sin(yaw)
    sin_pitch = math.sin(pitch)

    x = cos_roll * cos_yaw * cos_pitch + sin_roll * sin_pitch
    y = cos_roll * sin_yaw
    z = sin_roll * cos_yaw * cos_pitch - cos_roll * sin_pitch

    return np.array([x,y,z])

def find_intersection(vector, plane_z_pos):
    plane_normal = np.array([0,0,1])
    plane_point = np.array([0,0,plane_z_pos])

    dot_product = np.dot(vector, plane_normal)

    if np.isclose(dot_product, 0):return None
    
    vector_length_to_plane = ((np.dot(plane_point, plane_normal) - np.dot(np.array([0,0,0]), plane_normal)) / dot_product)

    if vector_length_to_plane < 0: return None

    return vector_length_to_plane * vector

def getObjectPosition(pixel_x, pixel_y, img_height, img_width, dist_from_camera=None, z_pos=None):
    # TODO: this is all wrong! euler angles cant be used, have to turn euler angle into vector and do math from there
    if dist_from_camera is not None: # ASSUMES FRONT CAMERA
        #first calculate the relative offset of the object from the center of the image (i.e. map pixel coordinates to values from -0.5 to 0.5)
        x_center_offset = (pixel_x-(img_width/2)) / img_width #-0.5 to 0.5
        y_center_offset = ((img_height/2)-pixel_y) / img_height #negated since y goes from top to bottom
        #use offset within image and total FOV of camera to find an angle offset from the angle the camera is facing
        #assuming FOV increases linearly with distance from center pixel
        yaw_angle_offset = front_cam_hfov*x_center_offset
        pitch_angle_offset = front_cam_vfov*y_center_offset
        
        direction_to_object = eulerToVector(0, pitch_angle_offset, yaw_angle_offset)
        vector_to_object = direction_to_object * dist_from_camera
        
        #convert local offsets to global offsets using tf transform library
        global_offset_x, global_offset_y, global_offset_z = transformLocalToGlobal(vector_to_object[0], vector_to_object[1], vector_to_object[2])
        x = state.x + global_offset_x
        y = state.y + global_offset_y
        z = state.z + global_offset_z
        x_conf = 1.0 - abs(x_center_offset)
        y_conf = 1.0 - abs(y_center_offset)
        pose_conf = x_conf*y_conf*(min(1.0, 5.0/dist_from_camera))
        return x,y,z, pose_conf
    elif z_pos is not None: # ASSUMES DOWN CAMERA
        #first calculate the relative offset of the object from the center of the image (i.e. map pixel coordinates to values from -0.5 to 0.5)
        x_center_offset = (pixel_x-(img_width/2)) / img_width #-0.5 to 0.5
        y_center_offset = ((img_height/2)-pixel_y) / img_height #negated since y goes from top to bottom
        #use offset within image and total FOV of camera to find an angle offset from the angle the camera is facing
        #assuming FOV increases linearly with distance from center pixel
        roll_angle_offset = down_cam_hfov*x_center_offset
        pitch_angle_offset = 90 + down_cam_vfov*y_center_offset

        local_direction_to_object = eulerToVector(roll_angle_offset, pitch_angle_offset, 0)
        global_direction_to_object = transformLocalToGlobal(local_direction_to_object[0], local_direction_to_object[1], local_direction_to_object[2])
        # solve for point that is defined by the intersection of the direction to the object and it's z position
        obj_pos = find_intersection(global_direction_to_object, z_pos)
        if obj_pos is None or np.linalg.norm(obj_pos - np.array([state.x, state.y, state.z])) > 10: return None, None, None, None

        x_conf = 1.0 - abs(x_center_offset)
        y_conf = 1.0 - abs(y_center_offset)
        x = state.x + obj_pos[0]
        y = state.y + obj_pos[1]
        z = z_pos
        pose_conf = x_conf * y_conf
        return x, y, z, pose_conf
    else:
        print("! ERROR: Not enough information to calculate a position! Require at least a known z position or a distance from the camera.")
        return None, None, None, None

def measureBuoyAngle(depth_cropped):
    return None

def measureGateAngle(depth_cropped):
    return None

def analyzeGate(img_cropped, debug_img):
    return 0

def analyzeBuoy(img_cropped, debug_img):
    return []


#one publisher per camera
cropped_img_pub = [
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
down_cam_hfov = 220
down_cam_vfov = 165.26
front_cam_hfov = 90
front_cam_vfov = 65

bridge = CvBridge()
tf_buffer = Buffer()
TransformListener(tf_buffer)
tf_header = Header(frame_id="world")
state = State()