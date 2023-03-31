import numpy as np
import rospy
from auv_msgs.msg import ObjectDetectionFrame
import math

observations = []

object_map = []

def objectDetectCb(msg):
    try:
        addObservation(msg)
        updateMap()
    except Exception as e:
        log(str(e))

def toObjects(msg):
    return [[msg.label[i], msg.obj_x[i], msg.obj_y[i], msg.obj_z[i], msg.obj_theta_z[i], msg.extra_field[i]] for i in range(len(msg.label))]

def addObservation(msg):
    global observations
    observations.append(msg)
    for i in range(len(msg.label)):
        if msg.label[i] == 0: #lane marker
            obj_i = findRelevantObject(msg)
            if obj_i == -1:
                object_map.append([msg.label[i], msg.obj_x[i], msg.obj_y[i], msg.obj_z[i], msg.obj_theta_z[i], msg.extra_field[i]])
            else:


def dist(obj1, obj2):


def getObjectsOfClass(i):
    objs = []
    for obj in object_map:
        if obj[0] == i: objs.append(obj)
    return objs
    
max_lane_markers = 2

max_gates = 1

max_buoys = 1

max_torpedo_target = 1

max_bins = 2

max_octagon = 1

if __name__ == '__main__':
    obj_sub = rospy.Subscriber('vision/viewframe_detection', ObjectDetectionFrame, objectDetectCb)