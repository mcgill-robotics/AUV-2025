#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion
import tf
from std_msgs.msg import Float64
from auv_msgs.msg import ObjectDetectionFrame
from auv_msgs.msg import ObjectMap
import numpy as np

def setup():
    global auv_marker
    # Create a marker message
    auv_marker = Marker()
    auv_marker.header.frame_id = "map"  # Set the frame ID according to your setup
    auv_marker.ns = "visualization"
    auv_marker.id = marker_id
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
    auv_marker.color.a = 0.8
    # Publish the marker
    auv_pub.publish(auv_marker)
    addHeading(0,0,0,1,0,0,auv_pub,(1,0,0))# 1 - AUV x direction
    addHeading(0,0,0,0,1,0,auv_pub,(0,1,0))# 2 - AUV y direction
    addHeading(0,0,0,0,0,1,auv_pub,(0,0,1))# 3 - AUV z direction
    addHeading(0,0,0,1,0,0,auv_pub,(1,1,1))# 4 - World x direction
    addHeading(0,0,0,0,1,0,auv_pub,(0,0,0))# 5 - World y direction
    addHeading(0,0,0,0,0,1,auv_pub,(0.5,0.5,0.5))# 6 - World z direction

def objectDetectCb(msg):
    #spawn blue spheres object detections
    for i in range(len(msg.label)):
        #NOTE: if performance becomes an issue, publish a marker array with all markers at once
        addDetectionMarker(msg.obj_x[i], msg.obj_y[i], msg.obj_z[i], 0.2, detection_pub, (0,0,1))

def objectMapCb(msg):
    global object_map_ids
    #spawn red spheres and text (for label, object-specific info) on objects in map
    for i in object_map_ids:
        removeMapMarkers(i)
    object_map_ids = []
    for i in range(len(msg.label)):
        addMapMarkers(msg.label[i], msg.obj_x[i], msg.obj_y[i], msg.obj_z[i], msg.theta_z[i], msg.extra_field[i])

def removeMapMarkers(marker_id):
    for i in range(4):
        marker_to_remove = Marker()
        marker_to_remove.header.frame_id = "map"
        marker_to_remove.id = marker_id + i
        marker_to_remove.action = Marker.DELETE
        map_sub.publish(marker_to_remove)    
    
def addDetectionMarker(x,y,z,scale,pub,color):
    global marker_id
    # Create a marker message
    detection_marker = Marker()
    detection_marker.header.frame_id = "map"  # Set the frame ID according to your setup
    detection_marker.ns = "visualization"
    marker_id += 1
    detection_marker.id = marker_id
    detection_marker.type = Marker.SPHERE
    detection_marker.action = Marker.ADD
    # Set the position, orientation, and scale
    detection_marker.pose.position = Point(x,y,z)
    detection_marker.pose.orientation = Quaternion(0, 0, 0, 1)
    detection_marker.scale.x = scale
    detection_marker.scale.y = scale
    detection_marker.scale.z = scale
    # Set the color (optional)
    detection_marker.color.r = color[0]
    detection_marker.color.g = color[1]
    detection_marker.color.b = color[2]
    detection_marker.color.a = 1.0
    # Publish the marker
    pub.publish(detection_marker)

def addHeading(x,y,z,theta_x,theta_y,theta_z,pub,color,override_id=None):
    global marker_id
    # Create a marker message for first heading
    heading_marker = Marker()
    heading_marker.header.frame_id = "map"  # Set the frame ID according to your setup
    heading_marker.ns = "visualization"
    if override_id is None:
        marker_id += 1
        heading_marker.id = marker_id
    else:
        heading_marker.id = override_id
    heading_marker.type = Marker.ARROW
    heading_marker.action = Marker.ADD
    x_offset, y_offset, z_offset = [theta_x,theta_y,theta_z] / np.linalg.norm([theta_x,theta_y,theta_z])
    heading_marker.points = [Point(x,y,z), Point(x+1.5*x_offset,y+1.5*y_offset,z+1.5*z_offset)]
    heading_marker.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(theta_x, theta_y, theta_z))
    heading_marker.scale.x = 0.1
    heading_marker.scale.y = 0.1
    heading_marker.scale.z = 0.1
    # Set the color (optional)
    heading_marker.color.r = color[0]
    heading_marker.color.g = color[1]
    heading_marker.color.b = color[2]
    heading_marker.color.a = 1.0
    # Publish the marker
    pub.publish(heading_marker)

def addMapMarkers(label,x,y,z,theta_z,extra_field):
    global marker_id
    global object_map_ids
    addDetectionMarker(x, y, z, 0.15, map_pub, (1,0,0))
    object_map_ids.append(marker_id)
    if label == 0: #LANE MARKER
        addHeading(x,y,z,cos(theta_z),sin(theta_z),0,map_pub,[0,1,0])
        addHeading(x,y,z,0,0,extra_field,map_pub,[0,1,0])
        addLabel(x,y,z,"Lane Marker",map_pub)
    elif label == 1: #QUALI GATE
        addHeading(x,y,z,cos(theta_z),sin(theta_z),0,map_pub,[0,1,0])
        addLabel(x,y,z,"Quali Gate",map_pub)
        marker_id += 1
    elif label == 2: #QUALI POLE
        addLabel(x,y,z,"Quali Pole",map_pub)
        marker_id += 1
        marker_id += 1
    elif label == 3: #GATE TASK
        addHeading(x,y,z,cos(theta_z),sin(theta_z),0,map_pub,[0,1,0])
        addLabel(x,y,z,"Gate: " + str(extra_field),map_pub)
        marker_id += 1
    elif label == 4: #BUOY TASK
        addHeading(x,y,z,cos(theta_z),sin(theta_z),0,map_pub,[0,1,0])
        addLabel(x,y,z,"Buoy: " + str(extra_field),map_pub)
        marker_id += 1

def addLabel(x,y,z,text,pub):
    global marker_id
    # Create a Marker message
    text_marker = Marker()
    text_marker.header.frame_id = "map"  # Modify with the appropriate frame ID
    text_marker.ns = "visualization"
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD
    marker_id += 1
    text_marker.id = marker_id

    # Set the position of the text marker
    text_marker.pose.position = Point(x,y,z)

    # Set the orientation of the text marker (identity quaternion)
    text_marker.pose.orientation.w = 1.0

    # Set the scale of the text marker (size in x, y, z)
    text_marker.scale.x = 0.2  # Modify with the desired scale of the text
    text_marker.scale.y = 0.2
    text_marker.scale.z = 0.2

    # Set the color of the text marker (RGBA)
    text_marker.color.r = 1.0
    text_marker.color.g = 1.0
    text_marker.color.b = 1.0
    text_marker.color.a = 1.0

    # Set the text value
    text_marker.text = text

    # Publish the Marker message
    pub.publish(marker)

def transformAuvToWorldRotation(x,y,z):
    pass

def updateAUVThetaX(msg):
    global auv_marker
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(auv_marker.pose.orientation.x, auv_marker.pose.orientation.y, auv_marker.pose.orientation.z, auv_marker.pose.orientation.w)
    new_quaternion = Quaternion(*tf.transformations.quaternion_from_euler(float(msg.data), pitch, yaw))
    auv_marker.pose.orientation = quaternion
    auv_pub.publish(auv_marker)
    updateReferenceFrames()
def updateAUVThetaY(msg):
    global auv_marker
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(auv_marker.pose.orientation.x, auv_marker.pose.orientation.y, auv_marker.pose.orientation.z, auv_marker.pose.orientation.w)
    new_quaternion = Quaternion(*tf.transformations.quaternion_from_euler(roll, float(msg.data), yaw))
    auv_marker.pose.orientation = quaternion
    auv_pub.publish(auv_marker)
    updateReferenceFrames()
def updateAUVThetaZ(msg):
    global auv_marker
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(auv_marker.pose.orientation.x, auv_marker.pose.orientation.y, auv_marker.pose.orientation.z, auv_marker.pose.orientation.w)
    new_quaternion = Quaternion(*tf.transformations.quaternion_from_euler(roll, pitch, float(msg.data)))
    auv_marker.pose.orientation = quaternion
    auv_pub.publish(auv_marker)
    updateReferenceFrames()
def updateAUVX(msg):
    global auv_marker
    auv_marker.pose.position = Point(float(msg.data), auv_marker.pose.position.y, auv_marker.pose.position.z)  # Set the desired position
    auv_pub.publish(auv_marker)
    updateReferenceFrames()
def updateAUVY(msg):
    global auv_marker
    auv_marker.pose.position = Point(auv_marker.pose.position.x, float(msg.data), auv_marker.pose.position.z)  # Set the desired position
    auv_pub.publish(auv_marker)
    updateReferenceFrames()
def updateAUVZ(msg):
    global auv_marker
    auv_marker.pose.position = Point(auv_marker.pose.position.x, auv_marker.pose.position.y, float(msg.data))  # Set the desired position
    auv_pub.publish(auv_marker)
    updateReferenceFrames()

def updateReferenceFrames():
    angle_x = transformAuvToWorldRotation(1,0,0)
    angle_y = transformAuvToWorldRotation(0,1,0)
    angle_z = transformAuvToWorldRotation(0,0,1)
    addHeading(auv_marker.pose.position.x,auv_marker.pose.position.y,auv_marker.pose.position.z,angle_x[0],angle_x[1],angle_x[2],auv_pub,(1,0,0),override_id=1)
    addHeading(auv_marker.pose.position.x,auv_marker.pose.position.y,auv_marker.pose.position.z,angle_y[0],angle_y[1],angle_y[2],auv_pub,(0,1,0),override_id=2)
    addHeading(auv_marker.pose.position.x,auv_marker.pose.position.y,auv_marker.pose.position.z,angle_z[0],angle_z[1],angle_z[2],auv_pub,(0,0,1),override_id=3)
    addHeading(auv_marker.pose.position.x,auv_marker.pose.position.y,auv_marker.pose.position.z,1,0,0,auv_pub,(1,1,1),override_id=4)
    addHeading(auv_marker.pose.position.x,auv_marker.pose.position.y,auv_marker.pose.position.z,0,1,0,auv_pub,(0,0,0),override_id=5)
    addHeading(auv_marker.pose.position.x,auv_marker.pose.position.y,auv_marker.pose.position.z,0,0,1,auv_pub,(0.5,0.5,0.5),override_id=6)

rospy.init_node('render_visualization')

auv_pub = rospy.Publisher('visualization/auv', Marker, queue_size=10)
detection_pub = rospy.Publisher('visualization/detection', Marker, queue_size=10)
map_pub = rospy.Publisher('visualization/map', Marker, queue_size=10)

print("Waiting 10 seconds so RViz can launch...")
rospy.sleep(10)
print("Starting visualization!")

object_map_ids = []
marker_id = 0
setup()

x_pos_sub = rospy.Subscriber('state_x', Float64, updateAUVX)
y_pos_sub = rospy.Subscriber('state_y', Float64, updateAUVY)
z_pos_sub = rospy.Subscriber('state_z', Float64, updateAUVZ)
theta_x_sub = rospy.Subscriber('state_theta_x', Float64, updateAUVThetaX)
theta_y_sub = rospy.Subscriber('state_theta_y', Float64, updateAUVThetaY)
theta_z_sub = rospy.Subscriber('state_theta_z', Float64, updateAUVThetaZ)
obj_sub = rospy.Subscriber('vision/viewframe_detection', ObjectDetectionFrame, objectDetectCb)
map_sub = rospy.Subscriber('vision/object_map', ObjectMap, objectMapCb)

rospy.spin()