#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo
import numpy as np
import os
import pickle

pwd = os.path.realpath(os.path.dirname(__file__))

#function to save camera calibration to file
def saveToFile(data, file):
    with open(pwd + "/" + file, 'wb') as f:
        pickle.dump(data, f)

#function to load camera calibration from file
def loadFromFile(file):
    try:
        with open(pwd + "/" + file, 'rb') as f:
            return pickle.load(f)
    except:
        return None

if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('stream_cameras')

    #get parameters from launch file
    sensor_id = rospy.get_param('~sensor_id')
    inputImgWidth = rospy.get_param('~inputImgWidth')
    inputImgHeight = rospy.get_param('~inputImgHeight')
    outputImgWidth = rospy.get_param('~outputImgWidth')
    outputImgHeight = rospy.get_param('~outputImgHeight')
    framerate = rospy.get_param('~framerate')
    distortion_model = rospy.get_param('~distortion_model', 'plumb_bob')
    
    #get parameters relative to down camera
    outputTopic = rospy.get_param('~outputTopic')
    camInfo = loadFromFile("camera_calibrations" + outputTopic + ".pickle")

    #define service callback for setting the camera info
    def setCameraInfo(req):
        global camInfo
        camInfo = req.camera_info
        saveToFile(camInfo, "camera_calibrations" + outputTopic + ".pickle")
        return True, "success"

    #define gstreamer pipeline to stream camera feeds
    pipeline = "nvarguscamerasrc sensor-id={} ! video/x-raw(memory:NVMM),width={},height={},framerate={}/1 ! nvvidconv ! video/x-raw(memory:NVMM),width={},height={},framerate={}/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink".format(sensor_id, inputImgWidth, inputImgHeight, framerate, outputImgWidth, outputImgHeight, framerate)
    #define service for setting camera info
    serv = rospy.Service("/vision" + outputTopic + '/set_camera_info', SetCameraInfo, setCameraInfo)
    #define output topics for the camera images
    pub = rospy.Publisher("/vision" + outputTopic + '/image_raw', Image, queue_size=1)
    #define output topics for the camera info
    camInfo_pub = rospy.Publisher("/vision" + outputTopic + '/camera_info', CameraInfo, queue_size=1)

    #define publishing function, takes as input one frame of the camera and publishes the camera info
    def publish(f):
        global camInfo
        msg = bridge.cv2_to_imgmsg(f, "bgr8")
        pub.publish(msg)
        #publish camera info
        if camInfo == None: #default values if not yet calibrated
            camInfo = CameraInfo()
            camInfo.height = msg.height
            camInfo.width = msg.width
            camInfo.distortion_model = distortion_model
            camInfo.D = np.zeros((5))
            camInfo.K = np.zeros((3,3))
            camInfo.R = np.zeros((3,3))
            camInfo.P = np.zeros((3,4))
        camInfo.header = msg.header
        camInfo_pub.publish(camInfo)
    
    #START GSTREAM using defined pipeline
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Failed to open camera")
        exit()
    #continuously read frames from the camera feed
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            print("Error reading frame from camera")
            break
        #publish each frame using the publishing function defined for the camera type
        publish(frame)
    cap.release()
        
