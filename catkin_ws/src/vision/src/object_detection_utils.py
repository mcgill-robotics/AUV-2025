import rospy
import cv2
from cv_bridge import CvBridge
from std_msgs.msgs import Float64
from sensor_msgs.msg import Image
import numpy as np
import math
from geometry_msgs.msg import Vector3, Vector3Stamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from std_msgs.msgs import Header
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

def getObjectPosition(pixel_x, pixel_y, img_height, img_width, dist_from_camera=None, z_pos=None):
    # TODO: THIS IS ALL WRONG! depth doesnt take into account the angle from center of the object
    # TODO: this is all wrong! euler angles cant be used, have to turn euler angle into vector and do math from there
    if dist_from_camera is not None: # ASSUMES FRONT CAMERA
        #first calculate the relative offset of the object from the center of the image (i.e. map pixel coordinates to values from -0.5 to 0.5)
        x_center_offset = (pixel_x-(img_width/2)) / img_width #-0.5 to 0.5
        y_center_offset = ((img_height/2)-pixel_y) / img_height #negated since y goes from top to bottom
        #use offset within image and total FOV of camera to find an angle offset from the angle the camera is facing
        #assuming FOV increases linearly with distance from center pixel
        x_angle_offset = front_cam_hfov*x_center_offset
        y_angle_offset = front_cam_vfov*y_center_offset
        #calculate slopes from angle offsets
        x_slope_offset = math.tan((x_angle_offset/180)*math.pi)
        y_slope_offset = math.tan((y_angle_offset/180)*math.pi)
        local_offset_x = dist_from_camera
        #use the angle and distance from the camera of the object to get x and y offsets in local space
        local_offset_y = local_offset_x*x_slope_offset
        local_offset_z = local_offset_x*y_slope_offset
        #convert local offsets to global offsets using tf transform library
        global_offset_x, global_offset_y, global_offset_z = transformLocalToGlobal(local_offset_x, local_offset_y, local_offset_z)
        x = state.x + global_offset_x
        y = state.y + global_offset_y
        z = state.z + global_offset_z
        x_conf = 1.0 - abs(x_center_offset)
        y_conf = 1.0 - abs(y_center_offset)
        pose_conf = x_conf*y_conf*(min(1.0, 1.0/dist_from_camera))
        return x,y,z, pose_conf
    elif z_pos is not None: # ASSUMES DOWN CAMERA
        #first calculate the relative offset of the object from the center of the image (i.e. map pixel coordinates to values from -0.5 to 0.5)
        x_center_offset = (pixel_x-(img_width/2)) / img_width #-0.5 to 0.5
        y_center_offset = ((img_height/2)-pixel_y) / img_height #negated since y goes from top to bottom
        #use offset within image and total FOV of camera to find an angle offset from the angle the camera is facing
        #assuming FOV increases linearly with distance from center pixel
        x_angle_offset = state.theta_x + down_cam_hfov*x_center_offset
        y_angle_offset = state.theta_y + down_cam_vfov*y_center_offset
        while abs(x_angle_offset) >= 180:
            if x_angle_offset >= 180: x_angle_offset -= 180
            else: x_angle_offset += 180

        while abs(y_angle_offset) >= 180:
            if y_angle_offset >= 180: y_angle_offset -= 180
            else: y_angle_offset += 180
        #check that the angle offsets face the object
        if state.z > z_pos:
            if abs(x_angle_offset) >= 90 or abs(y_angle_offset) >= 90: return None, None, None, None
            #calculate slopes from angle offsets
            x_slope_offset = math.tan((x_angle_offset/180)*math.pi)
            y_slope_offset = math.tan((y_angle_offset/180)*math.pi)
            #assume the object is at the bottom of the pool to get a depth
            global_offset_z = abs(state.z - z_pos)
            #use the angle and z offset from the object to get x and y offsets in local space
            local_offset_x = global_offset_z*x_slope_offset
            local_offset_y = global_offset_z*y_slope_offset
            #don't use position calculations for objects which are very far away
            # (in case down cam picks up on an object which is not on the floor of the pool)
            localization_max_distance = 10
            if abs(local_offset_x) > localization_max_distance or abs(local_offset_y) > localization_max_distance: return None, None, None, None
        else:
        #TODO!!!! if AUV is facing up
            if abs(x_angle_offset) <= 90 and abs(y_angle_offset) <= 90: return None, None, None, None
            if abs(x_angle_offset) >= 90:
                if x_angle_offset >= 180: x_angle_offset -= 180
                else: x_angle_offset += 180
            if abs(y_angle_offset) >= 90:
                if y_angle_offset >= 180: y_angle_offset -= 180
                else: y_angle_offset += 180
            #calculate slopes from angle offsets
            x_slope_offset = math.tan((x_angle_offset/180)*math.pi)
            y_slope_offset = math.tan((y_angle_offset/180)*math.pi)
            #assume the object is at the bottom of the pool to get a depth
            global_offset_z = -1 * abs(state.z - z_pos)
            #use the angle and z offset from the object to get x and y offsets in local space
            local_offset_x = global_offset_z*x_slope_offset
            local_offset_y = global_offset_z*y_slope_offset
            #don't use position calculations for objects which are very far away
            # (in case down cam picks up on an object which is not on the floor of the pool)
            localization_max_distance = 10
            if abs(local_offset_x) > localization_max_distance or abs(local_offset_y) > localization_max_distance: return None, None, None, None
                  
        #convert local offset to offset in world space using AUV yaw
        global_offset_x = local_offset_x*math.cos(math.radians(state.theta_z)) + local_offset_y*math.cos(math.radians(state.theta_z+90))
        global_offset_y = local_offset_y*math.sin(math.radians(state.theta_z+90)) + local_offset_x*math.sin(math.radians(state.theta_z))
                            
        #confidence model:
            # 0.25 at corners
            # 0.5 at edges 
            # 1.0 at center
        x_conf = 1.0 - abs(x_center_offset)
        y_conf = 1.0 - abs(y_center_offset)
        x = state.x + global_offset_x
        y = state.y + global_offset_y
        z = z_pos
        pose_conf = x_conf * y_conf
        return x,y,z, pose_conf
    else:
        print("! ERROR: Not enough information to calculate a position! Require at least a known z position or a dist_from_camera from the camera.")
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