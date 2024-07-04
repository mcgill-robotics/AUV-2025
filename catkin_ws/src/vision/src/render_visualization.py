#!/usr/bin/env python3

import rospy
import tf
import numpy as np
import math
from scipy.spatial.transform import Rotation

from auv_msgs.msg import VisionObjectArray, DeadReckonReport
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Quaternion, Wrench, Pose
from visualization_msgs.msg import Marker


def setup():
    global auv_marker
    # Create a marker message.
    auv_marker = Marker()
    auv_marker.header.frame_id = "world"  # Set the frame ID according to your setup.
    auv_marker.ns = "visualization"
    auv_marker.id = marker_id
    auv_marker.type = Marker.SPHERE
    auv_marker.action = Marker.ADD
    # Set the mesh file path.
    # marker.mesh_resource = "package://vision/src/visualization/auv.stl"
    # Set the position, orientation, and scale.
    auv_marker.pose.position = Point(0, 0, 0)
    auv_marker.pose.orientation = Quaternion(0, 0, 0, 1)
    auv_marker.scale.x = 0.9
    auv_marker.scale.y = 0.3
    auv_marker.scale.z = 0.3
    # Set the color (optional).
    auv_marker.color.r = 1.0
    auv_marker.color.g = 1.0
    auv_marker.color.b = 1.0
    auv_marker.color.a = 0.8
    # Publish the marker.
    pub_auv.publish(auv_marker)
    add_heading(0, 0, 0, 1, 0, 0, pub_auv.publish, (1, 0, 0))  # 1 - AUV x direction.
    add_heading(0, 0, 0, 0, 1, 0, pub_auv.publish, (0, 1, 0))  # 2 - AUV y direction.
    add_heading(0, 0, 0, 0, 0, 1, pub_auv.publish, (0, 0, 1))  # 3 - AUV z direction.
    add_heading(0, 0, 0, 1, 0, 0, pub_auv.publish, (1, 1, 1))  # 4 - World x direction.
    add_heading(0, 0, 0, 0, 1, 0, pub_auv.publish, (0, 0, 0))  # 5 - World y direction.
    add_heading(
        0, 0, 0, 0, 0, 1, pub_auv.publish, (0.5, 0.5, 0.5)
    )  # 6 - World z direction.
    add_heading(0, 0, 0, 1, 0, 0, pub_auv.publish, (1, 0, 1))  # 7 - DVL x direction.
    add_heading(0, 0, 0, 0, 1, 0, pub_auv.publish, (1, 1, 0))  # 8 - DVL y direction.
    add_heading(0, 0, 0, 0, 0, 1, pub_auv.publish, (0, 1, 1))  # 9 - DVL z direction.
    add_label(
        0,
        0,
        1,
        "Surge:0\nSway:0\nHeave:0\nRoll:0\nPitch:0\nYaw:0",
        pub_auv.publish,
        0.15,
    )  # 10 - effort.
    add_heading(-1000, 0, 0, 1, 0, 0, pub_auv.publish, (1, 1, 1))  # 11 - setpoint x axis.
    add_heading(-1000, 0, 0, 0, 1, 0, pub_auv.publish, (1, 1, 1))  # 12 - setpoint y axis.
    add_heading(-1000, 0, 0, 0, 0, 1, pub_auv.publish, (1, 1, 1))  # 13 - setpoint z axis.

    for gt in groundTruths:
        add_map_markers(gt[0], gt[1], gt[2], gt[3], gt[4], gt[5], (1, 1, 1))


def object_detect_cb(msg):
    # Spawn blue spheres object detections.
    for obj in msg.array:
        # NOTE: if performance becomes an issue, publish a marker 
        # array with all markers at once.
        add_detection_marker(obj.x, obj.y, obj.z)


def object_map_cb(msg):
    global updates, object_map_markers
    updates += 1
    if updates >= update_map_every:
        updates = 0
    else:
        return
    # Spawn red spheres and text (for label, object-specific info) on objects in map.
    for map_marker in object_map_markers:
        map_marker.action = Marker.DELETE
        pub_map.publish(map_marker)
        rospy.sleep(0.01)
    object_map_markers = []
    for obj in msg.array:
        add_map_markers(obj.label, obj.x, obj.y, obj.z, obj.theta_z, obj.extra_field)


def add_sphere(x, y, z, scale, pub, color):
    global marker_id
    # Create a marker message.
    detection_marker = Marker()
    detection_marker.header.frame_id = (
        "world"  # Set the frame ID according to your setup.
    )
    detection_marker.ns = "visualization"
    marker_id += 1
    detection_marker.id = marker_id
    detection_marker.type = Marker.SPHERE
    detection_marker.action = Marker.ADD
    # Set the position, orientation, and scale.
    detection_marker.pose.position = Point(x, y, z)
    detection_marker.pose.orientation = Quaternion(0, 0, 0, 1)
    detection_marker.scale.x = scale
    detection_marker.scale.y = scale
    detection_marker.scale.z = scale
    # Set the color (optional).
    detection_marker.color.r = color[0]
    detection_marker.color.g = color[1]
    detection_marker.color.b = color[2]
    detection_marker.color.a = 0.9
    # Publish the marker.
    pub(detection_marker)


def add_custom_object(marker_type, pos, rot, scale, pub, color):
    global marker_id
    # Create a marker message.
    custom_marker = Marker()
    custom_marker.header.frame_id = "world"  # Set the frame ID according to your setup.
    custom_marker.ns = "visualization"
    marker_id += 1
    custom_marker.id = marker_id
    custom_marker.type = marker_type
    custom_marker.action = Marker.ADD
    # Set the position, orientation, and scale.
    custom_marker.pose.position = Point(pos[0], pos[1], pos[2])
    custom_marker.pose.orientation = Quaternion(
        *tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
    )
    custom_marker.scale.x = scale[0]
    custom_marker.scale.y = scale[1]
    custom_marker.scale.z = scale[2]
    # Set the color (optional).
    custom_marker.color.r = color[0]
    custom_marker.color.g = color[1]
    custom_marker.color.b = color[2]
    custom_marker.color.a = color[3]
    # Publish the marker.
    pub(custom_marker)


def add_heading(x, y, z, vec_x, vec_y, vec_z, pub, color, override_id=None):
    global marker_id
    # Create a marker message for first heading.
    heading_marker = Marker()
    heading_marker.header.frame_id = "world"  # Set the frame ID according to your setup.
    heading_marker.ns = "visualization"
    if override_id is None:
        marker_id += 1
        heading_marker.id = marker_id
    else:
        heading_marker.id = override_id
    heading_marker.type = Marker.ARROW
    heading_marker.action = Marker.ADD
    x_offset, y_offset, z_offset = [vec_x, vec_y, vec_z] / np.linalg.norm(
        [vec_x, vec_y, vec_z]
    )
    heading_marker.points = [
        Point(x, y, z),
        Point(x + 0.55 * x_offset, y + 0.55 * y_offset, z + 0.55 * z_offset),
    ]
    heading_marker.pose.orientation = Quaternion(0, 0, 0, 1)
    heading_marker.scale.x = 0.035
    heading_marker.scale.y = 0.06
    heading_marker.scale.z = 0.035
    # Set the color (optional).
    heading_marker.color.r = color[0]
    heading_marker.color.g = color[1]
    heading_marker.color.b = color[2]
    heading_marker.color.a = 1.0
    # Publish the marker.
    pub(heading_marker)


def add_map_markers(label, x, y, z, in_theta_z, in_extra_field, color=(1, 0, 0)):
    global marker_id
    if in_extra_field == NULL_PLACEHOLDER:
        extra_field = -1
    else:
        extra_field = in_extra_field
    if in_theta_z == NULL_PLACEHOLDER:
        theta_z = 0
    else:
        theta_z = in_theta_z
    add_sphere(x, y, z, 0.1, publishToMap, color)
    if label == "Lane Marker":  # Lane marker.
        heading1 = euler_angle_to_unit_vector(0, 90, theta_z)
        heading2 = euler_angle_to_unit_vector(0, 90, extra_field)
        add_heading(x, y, z, heading1[0], heading1[1], heading1[2], publishToMap, color)
        add_heading(x, y, z, heading2[0], heading2[1], heading2[2], publishToMap, color)
        add_label(x, y, z, "Lane Marker", publishToMap)
    elif label == "Gate":  # Gate task.
        add_custom_object(
            Marker.CUBE,
            [x, y, z],
            (0, 0, theta_z * math.pi / 180),
            [0.1, 3, 1.5],
            publishToMap,
            [color[0], color[1], color[2], 0.4],
        )
        add_label(
            x,
            y,
            z,
            "Gate (left: "
            + str(
                "None"
                if extra_field < 0
                else ("earth" if extra_field > 0.5 else "abydos")
            )
            + ")",
            publishToMap,
        )
    elif label == "Buoy":  # Buoy task.
        add_custom_object(
            Marker.CUBE,
            [x, y, z],
            (0, 0, theta_z * math.pi / 180),
            [0.1, 1.2, 1.2],
            publishToMap,
            [color[0], color[1], color[2], 0.4],
        )
        add_label(x, y, z, "Buoy", publishToMap)
    elif label == "Octagon Table":  # Octagon table.
        add_custom_object(
            Marker.CUBE,
            [x, y, z],
            (0, 0, theta_z * math.pi / 180),
            [1.2, 1.2, 1.2],
            publishToMap,
            [color[0], color[1], color[2], 0.4],
        )
        add_label(x, y, z, "Octagon Table", publishToMap)
    elif label == "Earth Symbol":  # Earth symbol.
        add_custom_object(
            Marker.CUBE,
            [x, y, z],
            (0, 0, theta_z * math.pi / 180),
            [0.01, 0.2, 0.2],
            publishToMap,
            [color[0], color[1], color[2], 0.4],
        )
        add_label(x, y, z, "Earth", publishToMap)
    elif label == "Abydos Symbol":  # Abydos symbol.
        add_custom_object(
            Marker.CUBE,
            [x, y, z],
            (0, 0, theta_z * math.pi / 180),
            [0.01, 0.2, 0.2],
            publishToMap,
            [color[0], color[1], color[2], 0.4],
        )
        add_label(x, y, z, "Abydos", publishToMap)


def add_label(x, y, z, text, pub, scale=0.25, override_id=None):
    global marker_id
    # Create a Marker message.
    text_marker = Marker()
    text_marker.header.frame_id = "world"
    text_marker.ns = "visualization"
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD
    if override_id is None:
        marker_id += 1
        text_marker.id = marker_id
    else:
        text_marker.id = override_id

    if scale == 0.25:
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
    else:
        text_marker.color.r = 0.0
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0

    # Set the position of the text marker.
    text_marker.pose.position = Point(x, y, z + 0.5)

    # Set the orientation of the text marker (identity quaternion).
    text_marker.pose.orientation.w = 1.0

    # Set the scale of the text marker (size in x, y, z).
    text_marker.scale.x = scale  # Modify with the desired scale of the text.
    text_marker.scale.y = scale
    text_marker.scale.z = scale

    # Set the text value.
    text_marker.text = text

    # Publish the Marker message.
    pub(text_marker)


def transform_to_world_vector(vector, euler_angles):
    # Convert Euler angles to a rotation matrix.
    rotation = Rotation.from_euler("xyz", euler_angles, degrees=False)
    rotation_matrix = rotation.as_matrix()
    # Transform the direction vector to global coordinates.
    transformed_direction = np.matmul(rotation_matrix, vector)
    return transformed_direction.tolist()


def euler_angle_to_unit_vector(x, y, z):
    # Convert Euler angles to radians.
    roll_rad = math.radians(x)
    pitch_rad = math.radians(y)
    yaw_rad = math.radians(z)

    # Calculate the direction cosine matrix (DCM).
    cos_roll = math.cos(roll_rad)
    sin_roll = math.sin(roll_rad)
    cos_pitch = math.cos(pitch_rad)
    sin_pitch = math.sin(pitch_rad)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)

    # Define the DCM elements.
    vec = [
        sin_roll * sin_yaw + cos_roll * cos_yaw * sin_pitch,
        cos_roll * sin_pitch * sin_yaw - cos_yaw * sin_roll,
        cos_pitch * cos_roll,
    ]

    return vec


def publish_breadcrumb(marker):
    global breadcrumb_markers
    breadcrumb_markers.append(marker)
    pub_auv.publish(marker)


def add_breadcrumb(x, y, z):
    global breadcrumb_markers
    if len(breadcrumb_markers) >= max_breadcrumbs:
        breadcrumb_marker = breadcrumb_markers.pop(0)
        breadcrumb_marker.pose.position = Point(x, y, z)
        publish_breadcrumb(breadcrumb_marker)
    else:
        add_sphere(x, y, z, 0.02, publish_breadcrumb, (1, 0, 0))


def publish_detection_marker(marker):
    global detection_markers
    detection_markers.append(marker)
    pub_detection.publish(marker)


def add_detection_marker(x, y, z):
    global detection_markers
    if len(detection_markers) >= max_detection_markers:
        detection_marker = detection_markers.pop(0)
        detection_marker.pose.position = Point(x, y, z)
        publish_detection_marker(detection_marker)
    else:
        add_sphere(x, y, z, 0.075, publish_detection_marker, (0, 1, 0))


def update_auv_theta_x(msg):
    global auv_marker
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(
        [
            auv_marker.pose.orientation.x,
            auv_marker.pose.orientation.y,
            auv_marker.pose.orientation.z,
            auv_marker.pose.orientation.w,
        ]
    )
    new_quaternion = Quaternion(
        *tf.transformations.quaternion_from_euler(
            float(msg.data) * math.pi / 180, pitch, yaw
        )
    )
    auv_marker.pose.orientation = new_quaternion
    pub_auv.publish(auv_marker)
    update_reference_frames()


def update_auv_theta_y(msg):
    global auv_marker
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(
        [
            auv_marker.pose.orientation.x,
            auv_marker.pose.orientation.y,
            auv_marker.pose.orientation.z,
            auv_marker.pose.orientation.w,
        ]
    )
    new_quaternion = Quaternion(
        *tf.transformations.quaternion_from_euler(
            roll, float(msg.data) * math.pi / 180, yaw
        )
    )
    auv_marker.pose.orientation = new_quaternion
    pub_auv.publish(auv_marker)
    update_reference_frames()


def update_auv_theta_z(msg):
    global auv_marker
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(
        [
            auv_marker.pose.orientation.x,
            auv_marker.pose.orientation.y,
            auv_marker.pose.orientation.z,
            auv_marker.pose.orientation.w,
        ]
    )
    new_quaternion = Quaternion(
        *tf.transformations.quaternion_from_euler(
            roll, pitch, float(msg.data) * math.pi / 180
        )
    )
    auv_marker.pose.orientation = new_quaternion
    pub_auv.publish(auv_marker)
    update_reference_frames()


def update_auv_position(msg):
    global auv_marker
    auv_marker.pose.position = Point(
        float(msg.position.x), float(msg.position.y), float(msg.position.z)
    )  # Set the desired position
    add_breadcrumb(
        auv_marker.pose.position.x,
        auv_marker.pose.position.y,
        auv_marker.pose.position.z,
    )
    pub_auv.publish(auv_marker)
    update_reference_frames()
    add_label(
        auv_marker.pose.position.x,
        auv_marker.pose.position.y,
        auv_marker.pose.position.z + 1,
        "Surge:{}\nSway:{}\nHeave:{}\nRoll:{}\nPitch:{}\nYaw:{}".format(
            currentEffort["surge"],
            currentEffort["sway"],
            currentEffort["heave"],
            currentEffort["roll"],
            currentEffort["pitch"],
            currentEffort["yaw"],
        ),
        pub_auv.publish,
        0.15,
        override_id=10,
    )


def update_reference_frames():
    global auv_marker
    auv_euler_angles = tf.transformations.euler_from_quaternion(
        [
            auv_marker.pose.orientation.x,
            auv_marker.pose.orientation.y,
            auv_marker.pose.orientation.z,
            auv_marker.pose.orientation.w,
        ]
    )
    dir_x = transform_to_world_vector([1, 0, 0], auv_euler_angles)
    dir_y = transform_to_world_vector([0, 1, 0], auv_euler_angles)
    dir_z = transform_to_world_vector([0, 0, 1], auv_euler_angles)
    dvl_dir_x = transform_to_world_vector([1, 0, 0], dvl_euler_angles)
    dvl_dir_y = transform_to_world_vector([0, 1, 0], dvl_euler_angles)
    dvl_dir_z = transform_to_world_vector([0, 0, 1], dvl_euler_angles)
    add_heading(
        auv_marker.pose.position.x,
        auv_marker.pose.position.y,
        auv_marker.pose.position.z,
        dir_x[0],
        dir_x[1],
        dir_x[2],
        pub_auv.publish,
        (1, 0, 0),
        override_id=1,
    )
    add_heading(
        auv_marker.pose.position.x,
        auv_marker.pose.position.y,
        auv_marker.pose.position.z,
        dir_y[0],
        dir_y[1],
        dir_y[2],
        pub_auv.publish,
        (0, 1, 0),
        override_id=2,
    )
    add_heading(
        auv_marker.pose.position.x,
        auv_marker.pose.position.y,
        auv_marker.pose.position.z,
        dir_z[0],
        dir_z[1],
        dir_z[2],
        pub_auv.publish,
        (0, 0, 1),
        override_id=3,
    )
    add_heading(
        auv_marker.pose.position.x,
        auv_marker.pose.position.y,
        auv_marker.pose.position.z,
        1,
        0,
        0,
        pub_auv.publish,
        (1, 1, 1),
        override_id=4,
    )
    add_heading(
        auv_marker.pose.position.x,
        auv_marker.pose.position.y,
        auv_marker.pose.position.z,
        0,
        1,
        0,
        pub_auv.publish,
        (0, 0, 0),
        override_id=5,
    )
    add_heading(
        auv_marker.pose.position.x,
        auv_marker.pose.position.y,
        auv_marker.pose.position.z,
        0,
        0,
        1,
        pub_auv.publish,
        (0.5, 0.5, 0.5),
        override_id=6,
    )
    add_heading(
        auv_marker.pose.position.x,
        auv_marker.pose.position.y,
        auv_marker.pose.position.z,
        dvl_dir_x[0],
        dvl_dir_x[1],
        dvl_dir_x[2],
        pub_auv.publish,
        (1, 0, 1),
        override_id=7,
    )
    add_heading(
        auv_marker.pose.position.x,
        auv_marker.pose.position.y,
        auv_marker.pose.position.z,
        dvl_dir_y[0],
        dvl_dir_y[1],
        dvl_dir_y[2],
        pub_auv.publish,
        (1, 1, 0),
        override_id=8,
    )
    add_heading(
        auv_marker.pose.position.x,
        auv_marker.pose.position.y,
        auv_marker.pose.position.z,
        dvl_dir_z[0],
        dvl_dir_z[1],
        dvl_dir_z[2],
        pub_auv.publish,
        (0, 1, 1),
        override_id=9,
    )


def dvl_cb(msg):
    global dvl_euler_angles
    dvl_euler_angles = [
        float(msg.roll) * math.pi / 180,
        float(msg.pitch) * math.pi / 180,
        float(msg.yaw) * math.pi / 180,
    ]
    update_reference_frames()


def effort_cb(msg):
    global currentEffort
    currentEffort["surge"] = msg.force.x
    currentEffort["sway"] = msg.force.y
    currentEffort["heave"] = msg.force.z
    currentEffort["roll"] = msg.torque.x
    currentEffort["pitch"] = msg.torque.y
    currentEffort["yaw"] = msg.torque.z
    add_label(
        auv_marker.pose.position.x,
        auv_marker.pose.position.y,
        auv_marker.pose.position.z + 1,
        "Surge:{}\nSway:{}\nHeave:{}\nRoll:{}\nPitch:{}\nYaw:{}".format(
            currentEffort["surge"],
            currentEffort["sway"],
            currentEffort["heave"],
            currentEffort["roll"],
            currentEffort["pitch"],
            currentEffort["yaw"],
        ),
        pub_auv.publish,
        0.15,
        override_id=10,
    )


def setpoint_x_cb(msg):
    global current_setpoint
    current_setpoint[0] = msg.data
    update_setpoint_marker()


def setpoint_y_cb(msg):
    global current_setpoint
    current_setpoint[1] = msg.data
    update_setpoint_marker()


def setpoint_z_cb(msg):
    global current_setpoint
    current_setpoint[2] = msg.data
    update_setpoint_marker()


def setpoint_quat_cb(msg):
    global current_setpoint
    current_setpoint[3] = [msg.x, msg.y, msg.z, msg.w]
    update_setpoint_marker()


def update_setpoint_marker():
    setpoint_euler_angles = tf.transformations.euler_from_quaternion(
        [
            current_setpoint[3][0],
            current_setpoint[3][1],
            current_setpoint[3][2],
            current_setpoint[3][3],
        ]
    )
    dir_x = transform_to_world_vector([1, 0, 0], setpoint_euler_angles)
    dir_y = transform_to_world_vector([0, 1, 0], setpoint_euler_angles)
    dir_z = transform_to_world_vector([0, 0, 1], setpoint_euler_angles)
    add_heading(
        current_setpoint[0],
        current_setpoint[1],
        current_setpoint[2],
        dir_x[0],
        dir_x[1],
        dir_x[2],
        pub_auv.publish,
        (1, 0.8, 0),
        override_id=11,
    )
    add_heading(
        current_setpoint[0],
        current_setpoint[1],
        current_setpoint[2],
        dir_y[0],
        dir_y[1],
        dir_y[2],
        pub_auv.publish,
        (1, 0.8, 0),
        override_id=12,
    )
    add_heading(
        current_setpoint[0],
        current_setpoint[1],
        current_setpoint[2],
        dir_z[0],
        dir_z[1],
        dir_z[2],
        pub_auv.publish,
        (1, 0.8, 0),
        override_id=13,
    )

def publishToMap(marker):
    global object_map_markers
    object_map_markers.append(marker)
    pub_map.publish(marker)


if __name__ == "__main__":
    rospy.init_node("render_visualization")

    NULL_PLACEHOLDER = rospy.get_param("NULL_PLACEHOLDER")
    
    pub_auv = rospy.Publisher("visualization/auv", Marker, queue_size=999)
    pub_detection = rospy.Publisher("visualization/detection", Marker, queue_size=999)
    pub_map = rospy.Publisher("visualization/map", Marker, queue_size=999)

    # print("Waiting 10 seconds so RViz can launch...")
    # rospy.sleep(10)
    print("Starting visualization!")

    dvl_euler_angles = [0, 0, 0]
    breadcrumb_markers = []
    max_breadcrumbs = 500
    detection_markers = []
    max_detection_markers = 50
    object_map_markers = []
    current_setpoint = [0, 0, 0, [0, 0, 0, 1]]
    marker_id = 0

    update_map_every = 5
    updates = 0

    groundTruths = [
        # add objects here, format is [label,x,y,z,theta_z,extra_field]
    ]

    currentEffort = {"surge": 0, "sway": 0, "heave": 0, "roll": 0, "pitch": 0, "yaw": 0}

    setup()

    rospy.Subscriber("/state/pose", Pose, update_auv_position)
    rospy.Subscriber("/state/theta/x", Float64, update_auv_theta_x)
    rospy.Subscriber("/state/theta/y", Float64, update_auv_theta_y)
    rospy.Subscriber("/state/theta/z", Float64, update_auv_theta_z)
    rospy.Subscriber(
        "vision/viewframe_detection", VisionObjectArray, object_detect_cb
    )
    rospy.Subscriber("vision/object_map", VisionObjectArray, object_map_cb)
    rospy.Subscriber("/controls/effort", Wrench, effort_cb)
    rospy.Subscriber("/sensors/dvl/pose", DeadReckonReport, dvl_cb)
    rospy.Subscriber("x_setpoint", Float64, setpoint_x_cb)
    rospy.Subscriber("y_setpoint", Float64, setpoint_y_cb)
    rospy.Subscriber("z_setpoint", Float64, setpoint_z_cb)
    rospy.Subscriber(
        "/controls/quaternion_pid/setpoint", Quaternion, setpoint_quat_cb
    )

    rospy.spin()
