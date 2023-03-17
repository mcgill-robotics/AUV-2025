#!/usr/bin/env python3

import numpy as np
import rospy
import cv2
import pickle
from cv_bridge import CvBridge
import os
from os import listdir
from os.path import isfile, join
import re
import time
from sensor_msgs.msg import Image
from auv_msgs.msg import ObjectDetectionFrame

#callback for when a new image is received
def image_cb(raw_img, feed_name):
    global imgs
    global lastFrameTime
    #if time from when last image was received is greater than videoSaveTime we start a new video
    if time.time() - lastFrameTime.get(feed_name, time.time()+10) > videoSaveTime:
        save_images_to_video(shutdown=False)
    #save video in smaller increments (to avoid losing everything in case of crash)
    if len(imgs.get(feed_name, [])) > maxFramesPerVideo[feed_name]:
        save_images_to_video(shutdown=False)
    lastFrameTime[feed_name] = time.time()
    #convert sensor_msg image to cv2 image
    img = bridge.imgmsg_to_cv2(raw_img, "bgr8")
    #initialize image array
    if imgs.get(feed_name, None) == None:
        imgs[feed_name] = []
    imgs[feed_name].append(img)

#called when node shuts down
#saves all accumulated images into a new video file somewhere
def save_images_to_video(shutdown=True):
    global out
    #print("saving " + str(len(list(imgs.keys()))) + " videos to " + out)
    #for each feed
    for feed_name in list(imgs.keys()):
    	#don't bother saving videos less than 1 second long
        if len(imgs.get(feed_name)) < fps[feed_name]: continue
        #get names of all previously saved video files for this feed
        vid_filenames = [f for f in listdir(out) if isfile(join(out, f))]
        this_feed_filenames = [f for f in vid_filenames if feed_name in f]
        #get the largest number/id in the list of video files (we add 1 to this id to generate the new video filename)
        this_feed_filenames.append("-1") #in case there are no files, id will be set to -1 + 1 = 0
        last_used_video_id = max([int(re.search(r'\d+', f).group()) for f in this_feed_filenames])
        video_id = last_used_video_id + 1
        video_filename = out + "/" + feed_name + str(video_id)
        if shutdown: #if shutting down save as a pickle to go faster
            #write the video as a pickle file (faster) to file system
            #since otherwise writing a video takes too long and is SIGKILLed
            with open(video_filename + ".pickle") as f:
                pickle.dump(imgs[feed_name], f)
        else:
            #if not shutting down, write the video to file system as a video (slower)
            height, width, colors = imgs[feed_name][0].shape
            vid_out = cv2.VideoWriter(video_filename + ".mp4", cv2.VideoWriter_fourcc(*'mp4v'), fps[feed_name], (width, height))
            for i in imgs[feed_name]:
                vid_out.write(i)
            vid_out.release()
        #clear image array
        imgs[feed_name] = []

if __name__ == '__main__':
    #out is location where video is saved to
    out = os.path.realpath(os.path.dirname(__file__)) + "/auv_recordings"
    #make sure the folder exists
    if not os.path.exists(out):
        os.makedirs(out)
    imgs = {} #key = feed name, value = array of images
    lastFrameTime = {} #save last time a frame was received to automatically save video if feed is interrupted for a long period
    # initially a large value to make sure first frame is not saved seperately even if startup takes a while
    videoSaveTime = 2 #if 2 seconds go by with no frame received then save previous accumulated frames seperately
    bridge = CvBridge() #bridge is used to convert sensor_msg images to cv2 images
    rospy.init_node('record_feeds')
    #create subscriber callback for when new image is received on topic
    sub_down = rospy.Subscriber('vision/down_cam/image_raw', Image, image_cb, "down") #last argument is feed name
    sub_down_viz = rospy.Subscriber('vision/down_visual', Image, image_cb, "down_viz")
    fps = {"down":30, "down_viz":3} # fps of feeds
    maxFramesPerVideo = {feed:f*60 for feed, f in fps.items()} #one minute video increments
    #add a subscriber for each feed with different argument for feed name
    rospy.on_shutdown(save_images_to_video)
    rospy.spin()

        
