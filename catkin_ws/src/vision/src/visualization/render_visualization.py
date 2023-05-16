#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion
import os
import tf
from std_msgs.msg import Float64
from auv_msgs.msg import ObjectDetectionFrame
# from auv_msgs.msg import ObjectMap


def setup():
    global auv_marker
    # Create a marker message
    auv_marker = Marker()
    auv_marker.header.frame_id = "map"  # Set the frame ID according to your setup
    auv_marker.ns = "visualization"
    auv_marker.id = 0
    auv_marker.type = Marker.CUBE
    auv_marker.action = Marker.ADD
    # Set the mesh file path
    # marker.mesh_resource = "package://vision/src/visualization/auv.stl"
    # Set the position, orientation, and scale
    auv_marker.pose.position = Point(0, 0, 0)
    auv_marker.pose.orientation = Quaternion(0, 0, 0, 1)
    auv_marker.scale.x = 2.0
    auv_marker.scale.y = 1.0
    auv_marker.scale.z = 1.0
    # Set the color (optional)
    auv_marker.color.r = 1.0
    auv_marker.color.g = 1.0
    auv_marker.color.b = 1.0
    auv_marker.color.a = 1.0
    # Publish the marker
    auv_pub.publish(auv_marker)

def objectDetectCb(msg):
    #spawn blue spheres object detections
    for i in range(len(msg.label)):
        #NOTE: if performance becomes an issue, publish a marker array with all markers at once
        addDetection(msg.obj_x[i], msg.obj_y[i], msg.obj_z[i])

def addDetection(x,y,z):
    global marker_id
    # Create a marker message
    detection_marker = Marker()
    detection_marker.header.frame_id = "map"  # Set the frame ID according to your setup
    detection_marker.ns = "visualization"
    marker_id += 1
    detection_marker.id = marker_id
    detection_marker.type = Marker.SPHERE
    detection_marker.action = Marker.ADD
    # Set the mesh file path
    # marker.mesh_resource = "package://vision/src/visualization/auv.stl"
    # Set the position, orientation, and scale
    detection_marker.pose.position = Point(x,y,z)
    detection_marker.pose.orientation = Quaternion(0, 0, 0, 1)
    detection_marker.scale.x = 0.2
    detection_marker.scale.y = 0.2
    detection_marker.scale.z = 0.2
    # Set the color (optional)
    detection_marker.color.r = 0.0
    detection_marker.color.g = 0.0
    detection_marker.color.b = 1.0
    detection_marker.color.a = 1.0
    # Publish the marker
    detection_pub.publish(detection_marker)

def objectMap(msg):
    #spawn red spheres and text (for label, object-specific info) on objects in map
    pass

def updateAUVThetaX(msg):
    global auv_marker
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(auv_marker.pose.orientation.x, auv_marker.pose.orientation.y, auv_marker.pose.orientation.z, auv_marker.pose.orientation.w)
    new_quaternion = Quaternion(*tf.transformations.quaternion_from_euler(float(msg.data), pitch, yaw))
    auv_marker.pose.orientation = quaternion
    auv_pub.publish(auv_marker)
def updateAUVThetaY(msg):
    global auv_marker
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(auv_marker.pose.orientation.x, auv_marker.pose.orientation.y, auv_marker.pose.orientation.z, auv_marker.pose.orientation.w)
    new_quaternion = Quaternion(*tf.transformations.quaternion_from_euler(roll, float(msg.data), yaw))
    auv_marker.pose.orientation = quaternion
    auv_pub.publish(auv_marker)
def updateAUVThetaZ(msg):
    global auv_marker
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(auv_marker.pose.orientation.x, auv_marker.pose.orientation.y, auv_marker.pose.orientation.z, auv_marker.pose.orientation.w)
    new_quaternion = Quaternion(*tf.transformations.quaternion_from_euler(roll, pitch, float(msg.data)))
    auv_marker.pose.orientation = quaternion
    auv_pub.publish(auv_marker)
def updateAUVX(msg):
    global auv_marker
    auv_marker.pose.position = Point(float(msg.data), auv_marker.pose.position.y, auv_marker.pose.position.z)  # Set the desired position
    auv_pub.publish(auv_marker)
def updateAUVY(msg):
    global auv_marker
    auv_marker.pose.position = Point(auv_marker.pose.position.x, float(msg.data), auv_marker.pose.position.z)  # Set the desired position
    auv_pub.publish(auv_marker)
def updateAUVZ(msg):
    global auv_marker
    auv_marker.pose.position = Point(auv_marker.pose.position.x, auv_marker.pose.position.y, float(msg.data))  # Set the desired position
    auv_pub.publish(auv_marker)

rospy.init_node('render_visualization')
pwd = os.path.realpath(os.path.dirname(__file__))
auv_pub = rospy.Publisher('visualization/auv', Marker, queue_size=10)
detection_pub = rospy.Publisher('visualization/detection', Marker, queue_size=10)
map_pub = rospy.Publisher('visualization/map', Marker, queue_size=10)
print("Waiting 10 seconds so RViz can launch...")
rospy.sleep(10)
print("Starting visualization!")
marker_id = 0
setup()
x_pos_sub = rospy.Subscriber('state_x', Float64, updateAUVX)
y_pos_sub = rospy.Subscriber('state_y', Float64, updateAUVY)
z_pos_sub = rospy.Subscriber('state_z', Float64, updateAUVZ)
theta_x_sub = rospy.Subscriber('state_theta_x', Float64, updateAUVThetaX)
theta_y_sub = rospy.Subscriber('state_theta_y', Float64, updateAUVThetaY)
theta_z_sub = rospy.Subscriber('state_theta_z', Float64, updateAUVThetaZ)
obj_sub = rospy.Subscriber('vision/viewframe_detection', ObjectDetectionFrame, objectDetectCb)
# map_sub = rospy.Subscriber('vision/object_map', ObjectMap, objectMapCb)
rospy.spin()