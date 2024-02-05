#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from time import strftime
from tf import transformations
import math

def pose_callback(msg):
    global gps
    global roll, pitch, yaw
    global depth
    depth = msg.position.z
    gps = xyz_to_gps(msg.position.x, msg.position.y, msg.position.z)
    roll, pitch, yaw = transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

def image_callback(msg):
    global image
    data = bridge.imgmsg_to_cv2(msg)
    image = data

def xyz_to_gps(x, y, z):
    # This function will convert the x, y, z coordinates to GPS coordinates
    # The GPS coordinates will be returned as a tupl
    km_per_deg_lat, km_per_deg_long = get_gps_factors(z)
    latitude = x * 1000 / km_per_deg_lat + laditude_offset
    longitude = -y * 1000 / km_per_deg_long + longitude_offset
    return latitude, longitude

def init_text_file():
    global output_txt
    output_txt = open('data.txt', 'w')
    output_txt.write('Date Heure Latitude Longitude	Immersion Cap Pitch	Roll Easting_st	Northing_s Altitude_s\n')

def save_data(_):
    global gps 
    global depth
    global image
    global output_txt
    if gps is not None and image is not None:
        date = strftime("%d/%m/%Y")
        time = strftime("%H:%M:%S")
        output_txt.write(date + ' ' + time + ' ' + str(gps[0]) + ' ' + str(gps[1]) + ' ' + str(depth) + ' ' + str(yaw) + ' ' + str(pitch) + ' ' + str(roll) + ' ' + '1.0' + ' ' + '1.0' + ' ' + '1.0' + '\n') 


def get_gps_factors(depth):
    global radius_earth
    global laditude_offset
    global longitude_offset
    radius = radius_earth + depth
    lat_factor = radius * math.pi / 180
    long_factor = lat_factor * math.cos(math.radians(laditude_offset))
    return lat_factor, long_factor

def shutdown():
    global timer
    global output_txt
    output_txt.close()
    timer.shutdown()

if __name__ == '__main__':
    rospy.init_node('data_collection')

    gps = None
    depth = None
    roll, pitch, yaw = None, None, None
    image = None    
    bridge = CvBridge()
    radius_earth = rospy.get_param('~radius_earth')
    laditude_offset = rospy.get_param('~laditude_offset')
    longitude_offset = rospy.get_param('~longitude_offset')
    pose_sub = rospy.Subscriber('/state/pose', Pose, pose_callback)
    image_sub = rospy.Subscriber('/vision/down_cam/image_raw', Image, image_callback)
    init_text_file()
    update_rate = rospy.get_param('~update_rate')
    timer = rospy.Timer(rospy.Duration(1/update_rate), save_data)
    rospy.on_shutdown(shutdown)


    rospy.spin()