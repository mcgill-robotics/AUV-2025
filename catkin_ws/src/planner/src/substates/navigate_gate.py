#!/usr/bin/env python3

import rospy
import smach
from .utility.vision import *
import time, math
import numpy as np


class centerGate(smach.State):
    def __init__(self, control):
        super().__init__(outcomes=['success', 'failure'])
        if control == None: raise ValueError("control argument is None")
        self.control = control
        
    def execute(self, ud):
        '''
        Centers the gate until error is within tolerance.
        Once the gate is centered and scale, it returns success.
        If it loses sight of the gate for more than timeout, returns "failure".
        Assumes the gate is entirely in view (i.e. AUV is far enough to see the whole gate)
        '''
        print("Starting centering of gate.")
        global last_object_detection

        timeout = 5
        targetCenterX = 0.5
        targetCenterY = 0.5
        centering_tolerance = 0.1
        centered = False
        y_increment = 1
        z_increment = 1

        startTime = time.time()

        while not (centered):
            if len(last_object_detection) > 0:
                startTime = time.time()
                center_x = last_object_detection[0]
                center_y = last_object_detection[1]
                last_object_detection = []

                print("Gate in view at: x:{}, y:{}!".format(center_x, center_y))
                
                centering_error = math.sqrt((center_x - targetCenterX)**2 + (center_y - targetCenterY)**2)

                if centering_error > centering_tolerance: #isn't centered
                    delta_y = (targetCenterX - center_x)*y_increment
                    delta_z = (targetCenterY - center_y)*z_increment
                    print("Swaying {}".format(delta_y))
                    print("Heaving {}".format(delta_z))
                    self.control.moveDeltaLocal((0,delta_y,delta_z))
                    centered = False
                else:
                    centered = True
            else:
                print("No detection.... Timeout in " + str((startTime + timeout) - time.time()), end='\r')
                if time.time() > startTime + timeout:
                    print("Gate no longer visible.")
                    return 'failure'
                    
        print("Successfully centered gate!")
        return 'success'

class goThroughGate(smach.State):
    def __init__(self, control, extraSurge=0, extraRotation=(0,0,0)):
        super().__init__(outcomes=['success'])
        self.control = control
        self.extraSurge = extraSurge
        self.extraRotation = extraRotation
        
    def execute(self, ud):
        '''
        Moves the AUV forward until the gate is no longer in view.
        Once gate is no longer in view, it rotates the AUV by extraRotation.
        Once rotated, the AUV will move forward by extraSurge.
        '''
        global last_object_detection

        noDetectionTime = 2 #wait _ seconds before assuming gate no longer in view
        lastGateDetectionTime = time.time()
        surgeSpeed = 1

        print("Starting movement through gate.")
        self.control.forceLocal(surgeSpeed,0)
        while time.time() < lastGateDetectionTime + noDetectionTime:
            if len(last_object_detection) > 0:
                lastGateDetectionTime = time.time()
                last_object_detection = []
        self.control.forceLocal(0,0)
        print("Finished going through gate.")

        print("Rotating by {}.".format(self.extraRotation))
        self.control.rotateDelta(self.extraRotation)

        print("Surging by {}.".format(self.extraSurge))
        self.control.moveDeltaLocal((self.extraSurge, 0, 0))
        return 'success'

class goAroundPole(smach.State):
    def __init__(self, control):
        super().__init__(outcomes=['success'])
        self.control = control
        
    def execute(self, ud):
        '''
        Goes around a pole by doing 3 forward movements and 3 right angle turns.
        '''
        rotation = (0, 0, 90)
        surgeAmt = 1
        for i in range(3):
            print("Surging by {} then rotating by 90 degrees.".format(surgeAmt))
            self.control.moveDeltaLocal((surgeAmt, 0, 0))
            self.control.rotateDelta(rotation)

        return 'success'


#ignore unnecessary object detection information and make object detection event one array
#returns first object detection that is valid (assumes no duplicate gates in viewframe)
def parseMessage(msg):
    detection = []
    for i in range(len(msg.label)):
        if msg.camera == 0: #ignore detection on down camera
            continue
        if msg.label[i] != 1: #ignore detection of objects other than gate
            continue
        if len(detection) > 0 and detection[8] > msg.confidence[i]: #keep highest confidence prediction
            continue
        detection = [ msg.bounding_box_x[i],
            msg.bounding_box_y[i],
            msg.bounding_box_width[i],
            msg.bounding_box_height[i] ]
    return detection

last_object_detection = []
firstMessage = False

def objectDetectCb(msg):
    global last_object_detection
    global firstMessage
    firstMessage = True
    #in the future replace with finding gate nearby if any
    last_object_detection = parseMessage(msg)

class NavigateQualiGate(smach.State):
    def __init__(self, control=None):
        super().__init__(outcomes=['success'])
        if control == None: raise ValueError("Must pass in controller as control argument.")
        self.control = control
        
    def execute(self, ud):
        self.detector = ObjectDetector(msgHandler=objectDetectCb)
        self.detector.start()

        sm = smach.StateMachine(outcomes=['success', 'failure']) 
        with sm:
            smach.StateMachine.add('centerGate', centerGate(control=self.control), 
                    transitions={'success': 'goThroughGate', 'failure':'failure'})
            smach.StateMachine.add('goThroughGate', goThroughGate(control=self.control, extraRotation=(0,0,45), extraSurge=0), 
                    transitions={'success': 'goAroundPole'})
            smach.StateMachine.add('goAroundPole', goAroundPole(control=self.control), 
                    transitions={'success': 'centerGateAfterPole'})
            smach.StateMachine.add('centerGateAfterPole', centerGate(control=self.control), 
                    transitions={'success': 'goThroughGateFinal', 'failure':'failure'})
            smach.StateMachine.add('goThroughGateFinal', goThroughGate(control=self.control, extraRotation=(0,0,45), extraSurge=3), 
                    transitions={'success': 'success'})
        while not firstMessage: pass
        print("Received first detection message. Starting state machine.")
        res = sm.execute()
        self.detector.stop()
        del self.detector
        print("Ending state machine.")
