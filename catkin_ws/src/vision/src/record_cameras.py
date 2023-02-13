#!/usr/bin/env python3

import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
import os
from os import listdir
from os.path import isfile, join
import re
import time
from sensor_msgs.msg import Image
from auv_msgs.msg import ObjectDetectionFrame

#callback for when a new image is received
def image_cb(raw_img, camera_name):
    global imgs
    #if time from when last image was received is greater than videoSaveTime we start a new video
    if time.time() - lastFrameTime > videoSaveTime: save_images_to_video()
    lastFrameTime = time.time()
    #convert sensor_msg image to cv2 image
    img = bridge.imgmsg_to_cv2(raw_img, "bgr8")
    #initialize image array
    if imgs.get(camera_name, None) == None:
        imgs[camera_name] = []
    imgs[camera_name].append(img)

#called when node shuts down
#saves all accumulated images into a new video file somewhere
def save_images_to_video():
    print("saving " + str(len(list(imgs.keys()))) + " videos to " + out)
    #for each camera
    for camera_name in list(imgs.keys()):
        if len(imgs.get(camera_name)) == 0: continue
        #get names of all previously saved video files for this camera
        vid_filenames = [f for f in listdir(out) if isfile(join(out, f))]
        this_camera_filenames = [f for f in vid_filenames if camera_name in v]
        #get the largest number/id in the list of video files (we add 1 to this id to generate the new video filename)
        this_camera_filenames.append("-1") #in case there are no files, id will be set to -1 + 1 = 0
        last_used_video_id = max([int(re.search(r'\d+', f).group()) for f in this_camera_filenames])
        video_id = last_used_video_id + 1
        video_filename = camera_name + str(video_id) + ".mp4"
        #write the video to file system
        height, width, colors = imgs[camera_name][0].shape
        out = cv2.VideoWriter(out + "/" + video_filename, cv2.VideoWriter_fourcc(*'mp4v'), fps[camera_name], (width, height))
        for i in imgs[camera_name]:
            out.write(i)
        out.release()
        #clear image array
        imgs[camera_name] = []

if __name__ == '__main__':
    #out is location where video is saved to
    out = os.path.realpath(os.path.dirname(__file__)) + "/auv_recordings"
    #make sure the folder exists
    if not os.path.exists(out):
        os.makedirs(out)
    fps = {"down":30} # fps of cameras
    imgs = {} #key = camera name, value = array of images
    lastFrameTime = time.time()*10 #save last time a frame was received to automatically save video if feed is interrupted for a long period
    # initially a large value to make sure first frame is not saved seperately even if startup takes a while
    videoSaveTime = 2 #if 2 seconds go by with no frame received then save previous accumulated frames seperately
    bridge = CvBridge() #bridge is used to convert sensor_msg images to cv2 images
    rospy.init_node('record_cameras')
    #create subscriber callback for when new image is received on topic
    sub = rospy.Subscriber('vision/down_cam/image_raw', Image, image_cb, "down") #last argument is camera name
    #add a subscriber for each camera with different argument for camera name
    rospy.on_shutdown(save_images_to_video)
    rospy.spin()
