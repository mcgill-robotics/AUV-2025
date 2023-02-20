#!/usr/bin/env python3

import rospy
import smach
from auv_msgs.msg import ObjectDetectionFrame
from std_msgs.msg import Float64
import time

#move until a lane marker is detected
def findLaneMarker(timeout=30):
    startTime = time.time()
    forward = Float64(10.0)
    stop = Float64(0.0)
    pub = rospy.Publisher("surge", Float64, queue_size=50)

    #surge forward until we see a lane marker or reach timeout
    pub.publish(forward)
    while len(last_object_detection) < 1:
        if time.time() > startTime + timeout:
            print("Did not find lane marker in time.")
            pub.publish(stop)
            return 'failure'
    
    pub.publish(stop)
    return 'success'

def centerLaneMarker():
    pass

def rotateToHeading(timeout=30):
    global last_object_detection
    rotationError = 9999
    rotationTolerance = 5 #in degrees
    startTime = time.time()
    laneMarkerInView = False
    pub = rospy.Publisher('/theta_z_setpoint', Float64, queue_size=5, latch=True)

    while state_theta_z == None:
        if time.time() > startTime + timeout:
            print("Did not get state theta z update in time.")
            return 'failure'

    while abs(rotationError) > rotationTolerance:
        if len(last_object_detection) > 0:
            if rotationError == 9999:
                #the first time we assume that the angle pointing "downwards" (i.e. abs() above 90) is the angle we are coming from
                #so our target angle is the one closest to 0 degrees (most forward angle)
                rotationError = min(last_object_detection[4:6], key=lambda x : abs(x))
            else:
                #after the first time we assume our target heading is the one closest to our current theta_z_setpoint
                rotationError = min(last_object_detection[4:6], key=lambda x : abs(z_setpoint-(state_theta_z + x)))
            z_setpoint = state_theta_z + rotationError
            pub.publish(Float64(state_theta_z))
            startTime = time.time()
            last_object_detection = []
        else:
            if time.time() > startTime + timeout:
                print("Lane marker no longer sufficiently visible.")
                pub.publish(Float64(state_theta_z))
                return 'failure'

    pub.publish(Float64(state_theta_z))
    return 'success'

#ignore unnecessary object detection information and make object detection event one array
#returns first object detection that is valid (assumes no duplicate lane markers in viewframe)
def parseMessage(msg):
    for i in range(len(msg.camera)):
        if msg.camera[i] != 0: #ignore detection on stereo cameras
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
            msg.center_y[i]]
        return detection
    return []

def objectDetectCb(msg):
    global last_object_detection
    last_object_detection = parseMessage(msg)

def updateTheta(msg):
    global state_theta_z
    state_theta_z = float(msg.data)

if __name__ == '__main__':
    rospy.init_node('navigate_lane_marker')
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('find', findLaneMarker(),
                transitions={'success': 'center', 'failure':'failure'})
        smach.StateMachine.add('center', centerLaneMarker(), 
                transitions={'success': 'rotate', 'failure':'failure'})
        smach.StateMachine.add('rotate', rotateToHeading(), 
                transitions={'success': 'success', 'failure':'failure'})

    last_object_detection = []
    state_theta_z = None
    obj_sub = rospy.Subscriber('vision/viewframe_detection', ObjectDetectionFrame, objectDetectCb)
    theta_sub = rospy.Subscriber('state_theta_z', ObjectDetectionFrame, objectDetectCb)

    res = sm.execute()
