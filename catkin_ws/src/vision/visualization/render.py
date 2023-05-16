#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion
import os

def setup():
    # Create a marker message
    marker = Marker()
    marker.header.frame_id = "base_link"  # Set the frame ID according to your setup
    marker.ns = "auv_namespace"
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD

    # Set the mesh file path
    marker.mesh_resource = pwd + "/auv.stl"

    # Set the position, orientation, and scale
    marker.pose.position = Point(0, 0, 0)  # Set the desired position
    marker.pose.orientation = Quaternion(0, 0, 0, 1)  # Set the desired orientation
    marker.scale.x = 1.0  # Set the desired scale
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    # Set the color (optional)
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Publish the marker
    marker_pub.publish(marker)

    # Rotate the mesh
    # Set the desired rotation (Euler angles in radians)
    #roll = 0.0
    #pitch = 0.0
    #yaw = 1.57  # Rotate by 90 degrees (1.57 radians) around the Z-axis
    #quaternion = rospy.Quaternion(*rospy.get_quaternion_from_euler(roll, pitch, yaw))
    #marker.pose.orientation = quaternion
    #marker_pub.publish(marker)

    # Move the mesh
    #marker.pose.position = Point(1.0, 2.0, 3.0)  # Set the desired position
    #marker_pub.publish(marker)



rospy.init_node('render')
pwd = os.path.realpath(os.path.dirname(__file__))
marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
setup()