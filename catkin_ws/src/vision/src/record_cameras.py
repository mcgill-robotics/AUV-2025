#!/usr/bin/env python3

import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
import os
from os import listdir
from os.path import isfile, join
import re

from sensor_msgs.msg import Image
from auv_msgs.msg import ObjectDetectionFrame

def image_cb(raw_img, camera_name):
    global imgs
    img = bridge.imgmsg_to_cv2(raw_img, "bgr8")
    if imgs.get(camera_name, None) == None:
        imgs[camera_name] = []
    imgs[camera_name].append(img)

def save_images_to_video():
    pwd = os.path.realpath(os.path.dirname(__file__))
    print("saving " + str(len(imgs["downwards"])) + " images in " + pwd)
    for camera_name in list(imgs.keys()):
        if len(imgs.get(camera_name, [])) == 0: continue
        vid_filenames = [f for f in listdir(pwd + '/recordings') if isfile(join(pwd + '/recordings', f))]
        this_camera_filenames = [v for v in vid_filenames if camera_name in v]
        this_camera_filenames.append("-1") #in case there are no files
        last_used_video_id = max([int(re.search(r'\d+', f).group()) for f in this_camera_filenames])
        video_id = last_used_video_id + 1
        video_filename = camera_name + str(video_id) + ".mp4"
        height, width, colors = imgs[camera_name][0].shape
        out = cv2.VideoWriter(pwd + "/recordings/" + video_filename, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))
        for i in imgs[camera_name]:
            out.write(i)
        out.release()
        imgs[camera_name] = []
    


if __name__ == '__main__':
    fps = 30
    imgs = {}
    bridge = CvBridge()
    rospy.init_node('record_cameras')
    sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_cb, "downwards")
    #add a subscriber for each camera with different argument for camera name

    rospy.on_shutdown(save_images_to_video)
    rospy.spin()
