#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from time import strftime
from tf import transformations
import math
import cv2

def camera_info_callback(msg):
    global video
    global frame_rate
    global output_dir
    if video is not None: return
    width, height = msg.width, msg.height
    size = (width, height)
    title = strftime("%d/%m/%Y_%H:%M:%S")
    video = cv2.VideoWriter(output_dir + "/" + title + ".mp4", cv2.VideoWriter_fourcc(*'mp4v'), frame_rate, size)

def pose_callback(msg):
    global gps
    global roll, pitch, yaw
    global depth
    global seen_pose
    depth = msg.position.z
    gps = xyz_to_gps(msg.position.x, msg.position.y, msg.position.z)
    roll, pitch, yaw = transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w], axes='szyx')
    seen_pose = True

def image_callback(msg):
    global video
    global seen_image
    if video is None: return
    seen_image = True
    if not seen_pose: return
    data = bridge.imgmsg_to_cv2(msg)
    video.write(data)

def xyz_to_gps(x, y, z):
    # This function will convert the x, y, z coordinates to GPS coordinates
    # The GPS coordinates will be returned as a tupl
    km_per_deg_lat, km_per_deg_long = get_gps_factors(z)
    latitude = x * 1000 / km_per_deg_lat + laditude_offset
    longitude = -y * 1000 / km_per_deg_long + longitude_offset
    return latitude, longitude

def init_text_file():
    global output_txt
    global output_dir
    output_txt = open(output_dir + '/data.txt', 'w')
    output_txt.write('#date_dd/MM/yyyy,time,latitude,longitude,depth,altitude,pitch,roll\n')

def save_data(_):
    global gps 
    global depth
    global image
    global output_txt
    global roll, pitch, yaw
    global video
    print(seen_image, video, gps)
    if gps is not None and seen_image:
        date = strftime("%d/%m/%Y")
        time = strftime("%H:%M:%S")
        output_txt.write(date + ' ' + time + ' ' + str(gps[0]) + ' ' + str(gps[1]) + ' ' + str(depth) + ' ' + str(yaw) + ' ' + str(pitch) + ' ' + str(roll) + '\n') 


def get_gps_factors(depth):
    global radius_earth
    global laditude_offset
    global longitude_offset
    radius = radius_earth + depth/1000
    lat_factor = radius * math.pi / 180
    long_factor = lat_factor * math.cos(math.radians(laditude_offset))
    return lat_factor, long_factor

def shutdown():
    global video
    global timer
    global output_txt
    print("shutting down")
    output_txt.close()
    video.release()
    timer.shutdown()

if __name__ == '__main__':
    rospy.init_node('data_collection')

    gps = None
    depth = None
    roll, pitch, yaw = None, None, None
    seen_pose = False
    seen_image = False
    video = None
    bridge = CvBridge()
    radius_earth = rospy.get_param('~radius_earth')
    laditude_offset = rospy.get_param('~laditude_offset')
    longitude_offset = rospy.get_param('~longitude_offset')
    frame_rate = rospy.get_param('~frame_rate')
    output_dir = rospy.get_param('~output_dir')
    pose_sub = rospy.Subscriber('/state/pose', Pose, pose_callback)
    image_sub = rospy.Subscriber('/vision/down_cam/image_raw', Image, image_callback)
    camera_info_sub = rospy.Subscriber('/vision/down_cam/camera_info', CameraInfo, camera_info_callback)
    init_text_file()
    update_rate = rospy.get_param('~update_rate')
    timer = rospy.Timer(rospy.Duration(1/update_rate), save_data)
    try:
        rospy.spin()
    finally:
        shutdown()