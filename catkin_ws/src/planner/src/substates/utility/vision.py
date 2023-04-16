#!/usr/bin/env python3

import rospy
from auv_msgs.msg import ObjectDetectionFrame

#replace with radius + mapping once mapping is implemented
class ObjectDetector:
    def __init__(self, target_classes=[], callback=None, detectionHandler=None):
        if detectionhandler is None:
            detectionHandler = self.defaultDetectionCb
        try:
            len(cl)
            for cl in target_classes:
                int(cl)
        except:
            raise ValueError("target_class argument must be a list of integers")
        if callback == None and detectionHandler == self.defaultDetectionCb:
            raise ValueError("must pass at least one of callback or detectionHandler arguments")

        self.cb = callback
        self.target_classes = target_classes            
        #TEMPORARY UNTIL MAPPING IMPLEMENTED        
        self.consecutive_detections = 0
        self.min_consecutive_detections = 5
        self.detectionHandler = detectionHandler
    
    def start(self):
        #replace with map subscriber in future
        self.obj_sub = rospy.Subscriber('vision/viewframe_detection', ObjectDetectionFrame, self.detectionHandler)
    
    def stop(self):
        self.obj_sub.unregister()

    #replace with map handler in future
    def defaultDetectionCb(self, msg):
        #check for consecutive detections of a set of target classes (all classes if set is empty)
        detected = []
        for i in range(len(msg.label)):
            detected.append(msg.label[i])
            if len(self.target_classes) == 0 or msg.label[i] in self.target_classes:
                self.consecutive_detections[msg.label[i]] = self.consecutive_detections.get(msg.label[i], 0) + 1
                if self.consecutive_detections[msg.label[i]] >= self.min_consecutive_detections:
                    self.callback(msg)
                    break
        for k in self.consecutive_detections.keys():
            if k not in detected:
                self.consecutive_detections[k] = 0
