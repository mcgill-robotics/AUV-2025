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

def saveToFile(data, file):
    with open(pwd + "/" + file, 'wb') as f:
        pickle.dump(data, f)

def loadFromFile(file):
    try:
        with open(pwd + "/" + file, 'rb') as f:
            return pickle.load(f)
    except:
        return None

if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('stream_cameras')

    sensor_id = rospy.get_param('~sensor_id')
    inputImgWidth = rospy.get_param('~inputImgWidth')
    inputImgHeight = rospy.get_param('~inputImgHeight')
    outputImgWidth = rospy.get_param('~outputImgWidth')
    outputImgHeight = rospy.get_param('~outputImgHeight')
    framerate = rospy.get_param('~framerate')
    stereo = rospy.get_param('~stereo')
    distortion_model = rospy.get_param('~distortion_model', 'plumb_bob')
    
    if stereo:
        outputTopicLeft = rospy.get_param('~outputTopicLeft')
        outputTopicRight = rospy.get_param('~outputTopicRight')
        camInfoL = loadFromFile("camera_calibrations" + outputTopicLeft + ".pickle")
        camInfoR = loadFromFile("camera_calibrations" + outputTopicRight + ".pickle")

        def setLeftCameraInfo(req):
            global camInfoL
            camInfoL = req.camera_info
            saveToFile(camInfoL, "camera_calibrations" + outputTopicLeft + ".pickle")
            return True, "success"

        def setRightCameraInfo(req):
            global camInfoR
            camInfoR = req.camera_info
            saveToFile(camInfoR, "camera_calibrations" + outputTopicRight + ".pickle")
            return True, "success"

        pipeline = "nvarguscamerasrc sensor-id={} ! video/x-raw(memory:NVMM),width={},height={},framerate={}/1 ! nvvidconv ! video/x-raw(memory:NVMM),width={},height={},framerate={}/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink".format(sensor_id, inputImgWidth, inputImgHeight, framerate, outputImgWidth, outputImgHeight, framerate)
        servL = rospy.Service("/vision" + outputTopicLeft + '/set_camera_info', SetCameraInfo, setLeftCameraInfo)
        servR = rospy.Service("/vision" + outputTopicRight + '/set_camera_info', SetCameraInfo, setRightCameraInfo)
        pubL = rospy.Publisher("/vision" + outputTopicLeft + '/image_raw', Image, queue_size=1)
        pubR = rospy.Publisher("/vision" + outputTopicRight + '/image_raw', Image, queue_size=1)
        camInfo_pubL = rospy.Publisher("/vision" + outputTopicLeft + '/camera_info', CameraInfo, queue_size=1)
        camInfo_pubR = rospy.Publisher("/vision" + outputTopicRight + '/camera_info', CameraInfo, queue_size=1)
        
        def publish(f):
            global camInfoL
            global camInfoR
            #process frame to seperate into left and right images
            w = f.shape[1]
            fl = f[:, :int(w/2)-2]
            fr = f[:, int(w/2)-2:]
            msgL = bridge.cv2_to_imgmsg(fl, "bgr8")
            msgR = bridge.cv2_to_imgmsg(fr, "bgr8")
            pubL.publish(msgL)
            pubR.publish(msgR)
            #publish left camera info
            if camInfoL == None: #default values if not yet calibrated
                camInfoL = CameraInfo()
                camInfoL.height = msgL.height
                camInfoL.width = msgL.width
                camInfoL.distortion_model = distortion_model
                camInfoL.D = np.zeros((5))
                camInfoL.K = np.zeros((3,3))
                camInfoL.R = np.zeros((3,3))
                camInfoL.P = np.zeros((3,4))
            camInfoL.header = msgL.header
            camInfoL_pub.publish(camInfoL)
            #publish right camera info
            if camInfoR == None: #default values if not yet calibrated
                camInfoR = CameraInfo()
                camInfoR.height = msgR.height
                camInfoR.width = msgR.width
                camInfoR.distortion_model = distortion_model
                camInfoR.D = np.zeros((5))
                camInfoR.K = np.zeros((3,3))
                camInfoR.R = np.zeros((3,3))
                camInfoR.P = np.zeros((3,4))
            camInfoR.header = msgR.header
            camInfoR_pub.publish(camInfoR)
    else:
        outputTopic = rospy.get_param('~outputTopic')
        camInfo = loadFromFile("camera_calibrations" + outputTopic + ".pickle")

        def setCameraInfo(req):
            global camInfo
            camInfo = req.camera_info
            saveToFile(camInfo, "camera_calibrations" + outputTopic + ".pickle")
            return True, "success"

        pipeline = "nvarguscamerasrc sensor-id={} ! video/x-raw(memory:NVMM),width={},height={},framerate={}/1 ! nvvidconv ! video/x-raw(memory:NVMM),width={},height={},framerate={}/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink".format(sensor_id, inputImgWidth, inputImgHeight, framerate, outputImgWidth, outputImgHeight, framerate)
        serv = rospy.Service("/vision" + outputTopic + '/set_camera_info', SetCameraInfo, setCameraInfo)
        pub = rospy.Publisher("/vision" + outputTopic + '/image_raw', Image, queue_size=1)
        camInfo_pub = rospy.Publisher("/vision" + outputTopic + '/camera_info', CameraInfo, queue_size=1)

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
    

    #START GSTREAM
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Failed to open camera")
        exit()
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            print("Error reading frame from camera")
            break
        publish(frame)
    cap.release()
        
