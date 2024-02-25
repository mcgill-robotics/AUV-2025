#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from time import strftime
from tf import transformations
import math
import cv2
from pyproj import Transformer
from pyproj import CRS
from pyproj.aoi import AreaOfInterest
from pyproj.database import query_utm_crs_info
import time
import numpy as np
import quaternion

def camera_info_callback(msg):
    global video, frame_rat, output_dir, title
    if video is not None: return
    width, height = msg.width, msg.height
    size = (width, height)
    codec = cv2.VideoWriter_fourcc(*'MJPG')
    video = cv2.VideoWriter(output_dir + f"/{title}.avi", codec, frame_rate, size)

def pose_callback(msg):
    global gps, roll, pitch, yaw, depth, seen_pose, backwards, north_offset, east_offset
    new_north, new_east = north_offset + msg.position.x, east_offset - msg.position.y
    gps = backwards.transform(new_north,  new_east)
    depth = msg.position.z
    q_nwu_auv = np.quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
    q_ned_auv = np.quaternion(0,1,0,0) * q_nwu_auv * np.quaternion(0,1,0,0)
    yaw, pitch, roll = transformations.euler_from_quaternion([q_ned_auv.x, q_ned_auv.y, q_ned_auv.z, q_ned_auv.w], axes='rzyx')
    roll *= 180 / math.pi
    pitch *= 180 / math.pi
    yaw *= 180 / math.pi
    seen_pose = True

def image_callback(msg):
    global image, seen_image
    if video is None: return
    seen_image = True
    if not seen_pose: return
    data = bridge.imgmsg_to_cv2(msg, "bgr8")
    image = data

# def xyz_to_gps(x, y, z):
#     # This function will convert the x, y, z coordinates to GPS coordinates
#     # The GPS coordinates will be returned as a tupl
#     km_per_deg_lat, km_per_deg_long = get_gps_factors(z)
#     latitude = x / 1000 / km_per_deg_lat + laditude_offset
#     longitude = -y / 1000 / km_per_deg_long + longitude_offset
#     return latitude, longitude

def init_text_file():
    global output_txt, output_dir, title
    output_txt = open(output_dir + f'/{title}.txt', 'w')
    output_txt.write('##date_dd/MM/yyyy,time,latitude,longitude,depth,heading,pitch,roll\n')

def save_data(_):
    global gps, depth, image, output_txt, roll, pitch, yaw, video
    if gps is not None and seen_image:
        date = strftime("%d/%m/%Y")
        millis = str(int(round(time.time() * 1000)))[0:3]
        the_time = strftime(f"%H:%M:%S.{millis}")
        output_txt.write(f"{date},{the_time},{gps[0]:10.15f},{gps[1]:10.15f},{depth:10.9f},{yaw:10.9f},{pitch:10.9f},{roll:10.9f}\n")
        video.write(image)


# def get_gps_factors(depth):
#     global radius_earth
#     global laditude_offset
#     global longitude_offset
#     radius = radius_earth + depth/1000
#     lat_factor = radius * math.pi / 180
#     long_factor = lat_factor * math.cos(math.radians(laditude_offset))
#     return lat_factor, long_factor

def shutdown():
    global video
    global timer
    global output_txt
    print("shutting down")
    cv2.destroyAllWindows()
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
    image = None
    video = None
    title = strftime("%d_%m_%Y_%H:%M:%S")
    bridge = CvBridge()
    laditude_offset = rospy.get_param('~laditude_offset')
    longitude_offset = rospy.get_param('~longitude_offset')
    frame_rate = rospy.get_param('~frame_rate')
    output_dir = rospy.get_param('~output_dir')

    utm_crs_list = query_utm_crs_info(
            datum_name="WGS 84",
            area_of_interest=AreaOfInterest(
                west_lon_degree=longitude_offset,
                south_lat_degree=laditude_offset,
                east_lon_degree=longitude_offset,
                north_lat_degree=laditude_offset,
            ),
        )
    utm_crs = CRS.from_epsg(utm_crs_list[0].code)
    forwards = Transformer.from_crs("EPSG:4326", utm_crs, always_xy=True)
    backwards = Transformer.from_crs(utm_crs, "EPSG:4326", always_xy=True)
    north_offset, east_offset = forwards.transform(laditude_offset, laditude_offset)


    pose_sub = rospy.Subscriber('/state/pose', Pose, pose_callback)
    image_sub = rospy.Subscriber('/vision/down_cam/image_raw', Image, image_callback)
    camera_info_sub = rospy.Subscriber('/vision/down_cam/camera_info', CameraInfo, camera_info_callback)
    init_text_file()
    timer = rospy.Timer(rospy.Duration(1/frame_rate), save_data)
    try:
        rospy.spin()
    finally:
        shutdown()