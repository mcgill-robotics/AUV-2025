import numpy as np
import rospy
from auv_msgs.msg import ObjectDetectionFrame
import math

def objectDetectCb(msg):
    try:
        print(msg)
    except Exception as e:
        log(str(e))


if __name__ == '__main__':
    obj_sub = rospy.Subscriber('vision/viewframe_detection', ObjectDetectionFrame, objectDetectCb)