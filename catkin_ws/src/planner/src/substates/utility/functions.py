import math
from tf import transformations
import numpy as np
import quaternion
import rospy
from std_msgs.msg import String

def countdown(secs):
    pub_mission_display = rospy.Publisher("/mission_display", String, queue_size=1)
    end_time = rospy.get_time() + secs
    while rospy.get_time() < end_time and not rospy.is_shutdown():
        pub_mission_display.publish(f"T - {end_time - rospy.get_time():.3f}")


def degreesToVector(yawDegrees):
    angleRadians = yawDegrees * math.pi / 180
    x = math.cos(angleRadians)
    y = math.sin(angleRadians)
    return [x, y]


def vectorToYawDegrees(x, y):
    angle_radians = math.atan2(y,x)
    return math.degrees(angle_radians)
    # zero_angle_vector = np.array([1, 0])
    # arg_vector = np.array([x, y])
    # magnitude_arg_vector = np.linalg.norm(arg_vector)
    # dot_product = np.dot(zero_angle_vector, arg_vector)
    # return math.acos(dot_product / magnitude_arg_vector) * 180 / math.pi


def normalize_vector(vector2D):
    magnitude = math.sqrt(vector2D[0] ** 2 + vector2D[1] ** 2)
    if magnitude == 0:
        return (0, 0)
    normalized_x = vector2D[0] / magnitude
    normalized_y = vector2D[1] / magnitude
    return [normalized_x, normalized_y]


def dotProduct(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]


def quaternion_between_vectors(v1, v2):
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    dot = np.dot(v1, v2)
    if np.isclose(dot, 1.0):  #checking for colinearity
        return np.quaternion(1.0, 0.0, 0.0, 0.0)  
    elif np.isclose(dot, -1.0): 
        return np.quaternion(0.0, 0.0, 0.0, 1.0)  #180 about Z
    else:
        axis = np.cross(v1, v2)
        axis /= np.linalg.norm(axis)
        angle = np.arccos(dot)
        return quaternion.from_rotation_vector(axis * angle)


def euler_to_quaternion(roll, pitch, yaw):
    roll_radians = math.radians(roll)
    pitch_radians = math.radians(pitch)
    yaw_radians = math.radians(yaw)
    q = transformations.quaternion_from_euler(yaw_radians, pitch_radians, roll_radians, "rzyx")
    return [q[0], q[1], q[2], q[3]] #quats are represented as x,y,z,w. Should check this in all codebase


# def countdown(secs):
#     pub_mission_display = rospy.Publisher("/mission_display", String, queue_size=1)
#     end_time = rospy.get_time() + secs
#     while rospy.get_time() < end_time:
#         pub_mission_display.publish(str(end_time - rospy.get_time()))
