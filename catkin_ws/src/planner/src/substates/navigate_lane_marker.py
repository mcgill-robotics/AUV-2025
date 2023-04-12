#!/usr/bin/env python3

import rospy
import smach
from utility import *
import time, math
import numpy as np

class centerAndScale(smach.State):
    def __init__(self):
        super().__init__(outcomes=['success', 'failure'])
        
    def execute(self, ud):
        '''
        Centers and scales the lane marker until error is within tolerance.
        Once the lane marker is centered and scale, it returns success.
        If it loses sight of the lane marker for more than timeout, returns "failure".
        '''
        print("Starting centering and scaling of lane marker.")
        global last_object_detection

        timeout = 5
        targetCenterX = 0.5
        targetCenterY = 0.5
        targetScale = 0.5
        centering_tolerance = 0.1
        scaling_tolerance = 0.1
        centered = False
        scaled = False
        z_increment = 1
        x_increment = 1
        y_increment = 1

        startTime = time.time()

        while not (centered and scaled):
            if len(last_object_detection) > 0:
                startTime = time.time()
                center_x = last_object_detection[6]
                center_y = last_object_detection[7]
                width = last_object_detection[2]
                height = last_object_detection[3]
                last_object_detection = []

                print("Lane marker in view at: x:{}, y:{}, w:{}, h:{}!".format(center_x, center_y, width, height))

                scaling_error = max(width, height) - targetScale
                
                if abs(scaling_error) > scaling_tolerance:
                    delta_z = scaling_error*z_increment
                    print("Moving z by {}".format(delta_z))
                    controller.moveDeltaLocal((0,0,delta_z))
                    scaled = False
                    continue
                else:
                    scaled = True
                
                centering_error = math.sqrt((center_x - targetCenterX)**2 + (center_y - targetCenterY)**2)

                if centering_error > centering_tolerance: #isn't centered
                    delta_y = (targetCenterX - center_x)*y_increment
                    delta_x = (targetCenterY - center_y)*x_increment
                    print("Surging {}".format(delta_x))
                    print("Swaying {}".format(delta_y))
                    controller.moveDeltaLocal((delta_x,delta_y,0))
                    centered = False
                    continue
                else:
                    centered = True
            else:
                print("No detection.... Timeout in " + str((startTime + timeout) - time.time()), end='\r')
                if time.time() > startTime + timeout:
                    print("Lane marker no longer visible.")
                    return 'failure'
                    
        print("Successfully positioned above lane marker!")
        return 'success'

class measureLaneMarker(smach.State):
    def __init__(self):
        super().__init__(outcomes=['success', 'failure'])
        
    def execute(self, ud):
        '''
        Rotates the AUV until lane marker target heading is within rotational tolerance.
        Once the lane marker heading is facing forward, it returns success.
        If it loses sight of the lane marker for more than timeout, returns "failure".
        Assumes that the most "downward" heading is the one where it comes from. (once mapping is implemented this should be improved)
        '''
        print("Starting measurement of lane marker headings.")
        global last_object_detection
        global target_heading
        measurements = []
        min_measurements = 25
        startTime = time.time()
        timeout = 5

        #get measurements
        while len(measurements) < min_measurements:
            if len(last_object_detection) > 0:
                if abs(last_object_detection[4]) < abs(last_object_detection[5]):
                    measurements.append(last_object_detection[4])
                else:
                    measurements.append(last_object_detection[5])
                startTime = time.time()
                last_object_detection = []
            else:
                print("No detection.... Timeout in " + str((startTime + timeout) - time.time()), end='\r')
                if time.time() > startTime + timeout:
                    print("Lane marker no longer visible.")
                    return 'failure'

        #remove outliers with median
        target_heading = np.median(measurements)
        
        return 'success'

class rotateToHeading(smach.State):
    def __init__(self):
        super().__init__(outcomes=['success', 'failure'])
        
    def execute(self, ud):
        '''
        Rotates the AUV until lane marker target heading is within rotational tolerance.
        Once the lane marker heading is facing forward, it returns success.
        If it loses sight of the lane marker for more than timeout, returns "failure".
        Assumes that the most "downward" heading is the one where it comes from. (once mapping is implemented this should be improved)
        '''
        if target_heading is None: return 'failure'
        print("Starting rotation of AUV to target heading.")
        controller.rotateDelta((0,0,target_heading))
        print("Successfully rotated to lane marker!")
        return 'success'

#ignore unnecessary object detection information and make object detection event one array
#returns first object detection that is valid (assumes no duplicate lane markers in viewframe)
def parseMessage(msg):
    detection = []
    for i in range(len(msg.label)):
        if len(detection) > 0 and detection[8] > msg.confidence[i]: #keep highest confidence prediction
            continue
        if msg.camera != 0: #ignore detection on stereo cameras
            continue
        if msg.label[i] != 0: #ignore detection of objects other than lane marker
            continue
        if msg.headings1[i] == None or msg.headings2[i] == None: #ignore detection that did not glean heading information
            continue
        if msg.center_x[i] == None or msg.center_y[i] == None: #ignore detection that did not glean center point information
            continue
        detection = [ msg.bounding_box_x[i],
            msg.bounding_box_y[i],
            msg.bounding_box_width[i],
            msg.bounding_box_height[i],
            msg.heading1[i]-90, #-90 because 90 degrees orientation on unit circle is 0 degrees for AUV
            msg.heading2[i]-90,
            msg.center_x[i],
            msg.center_y[i],
            msg.confidence[i]]
    return detection

target_heading = None
last_object_detection = []
firstMessage = False

def objectDetectCb(msg):
    global last_object_detection
    global firstMessage
    firstMessage = True
    #in the future replace with finding lane marker nearby if any
    last_object_detection = parseMessage(msg)

class NavigateLaneMarker(smach.State):
    def __init__(self):
        super().__init__(outcomes=['success'])
        
    def execute(self, ud):
        self.detector = vision.ObjectDetector(msgHandler=objectDetectCb)
        self.detector.start()

        sm = smach.StateMachine(outcomes=['success', 'failure']) 
        with sm:
            smach.StateMachine.add('scale_and_center', centerAndScale(), 
                    transitions={'success': 'measure', 'failure':'failure'})
            smach.StateMachine.add('measure', measureLaneMarker(), 
                    transitions={'success': 'rotate', 'failure':'failure'})
            smach.StateMachine.add('rotate', rotateToHeading(), 
                    transitions={'success': 'success', 'failure':'failure'})
        while not firstMessage: pass
        print("Received first detection message. Starting state machine.")
        res = sm.execute()
        self.detector.stop()
        print("Ending state machine.")
