#!/usr/bin/env python3

import rospy
import smach
from auv_msgs.msg import ObjectDetectionFrame
from std_msgs.msg import Float64
import time, math

targetScale = 0.5
targetCenterX = 0.5
targetCenterY = 0.5
heave_p_value = 10
surge_p_value = 10
sway_p_value = 10


#move until a lane marker is detected
def findLaneMarker(timeout=30):
    startTime = time.time()
    forward = Float64(10.0)
    stop = Float64(0.0)
    pub = rospy.Publisher("surge", Float64, queue_size=50)

    #surge forward until we see a lane marker or reach timeout
    pub.publish(forward)
    print("moving forward")
    while len(last_object_detection) < 1:
        if time.time() > startTime + timeout:
            print("Did not find lane marker in time.")
            pub.publish(stop)
            return 'failure'
    print("lane marker in view!")
    pub.publish(stop)
    return 'success'



def isScaled():
    '''
    Return True if the current scale is equal to the target scale within a tolerance
    Else return False
    '''
    global last_object_detection
    scalingTolerance = 0.05 #ratio

    if len(last_object_detection) > 0:
        width = last_object_detection[2]
        height = last_object_detection[3]
        currentScale = max(width, height)
        scalingError = abs(currentScale - targetScale)

        return abs(scalingError) < scalingTolerance

    return False

def scaleLaneMarker(timeout=5):

    '''
    Scale the lane marker by publishing a heave value 
    '''
    global last_object_detection
    
    startTime = time.time()
    laneMarkerInView = False
    success = False
    pub = rospy.Publisher('/heave', Float64, queue_size=5, latch=True)
    
    
    if len(last_object_detection) > 0:
        
        
        width = last_object_detection[2]
        height = last_object_detection[3]
        currentScale = max(width, height)
        heaveValue = (currentScale - targetScale)*heave_p_value
        
        pub.publish(Float64(heaveValue))
        startTime = time.time()
        last_object_detection = []
    else:
        print("No detection.... Timeout in " + str((startTime + timeout) - time.time()), end='\r')
        if time.time() > startTime + timeout:
            print("Lane marker no longer visible.")
            pub.publish(Float64(0))
            return 'failure'

        
    
    print("Successfully rotated to lane marker!")
    pub.publish(Float64(0))
    return 'success'


def isCentered():
    '''
    Return True if the distance between (center_x, center_y) and (targetCenterX, targetCenterY) 
    is smaller than the centering tolerance, else return False
    '''
    global last_object_detection
    centeringTolerance = 0.05
    if len(last_object_detection) > 0:
        
        center_x = last_object_detection[6]
        center_y = last_object_detection[7]

        return math.sqrt((center_x - targetCenterX)**2 + (center_y - targetCenterY)**2) < centeringTolerance

    return False

    
    
    
    

def centerLaneMarker(timeout=5):    

    '''
    Center the lane marker by publishing a sway value and a surge value
    '''
    global last_object_detection
    
    startTime = time.time()
    pubx = rospy.Publisher('/surge', Float64, queue_size=5, latch=True)
    puby = rospy.Publisher('/sway', Float64, queue_size=5, latch=True)
    
    if len(last_object_detection) > 0:
        
        center_x = last_object_detection[6]
        center_y = last_object_detection[7]

        surgeValue = (targetCenterX - center_x)*surge_p_value
        swayValue = (targetCenterY - center_y)*sway_p_value

        pubx.publish(Float64(surgeValue))
        puby.publish(Float64(swayValue))
        
        startTime = time.time()
        last_object_detection = []
    else:
        print("No detection.... Timeout in " + str((startTime + timeout) - time.time()), end='\r')
        if time.time() > startTime + timeout:
            pubx.publish(Float64(0))
            puby.publish(Float64(0))
            return 'failure'

        
    
    print("Successfully centered to lane marker!")
    pubx.publish(Float64(0))
    puby.publish(Float64(0))
    return 'success'



def centerAndScale():
    '''
    If the lane marker is centered and scale, it return succes.
    Otherwise, it centers then scales the lane marker.
    If one of the two actions return failure, it returns failure.
    If not, it call itself recursively.
    '''
    if isCentered() and isScaled():
        return 'succes'
    
    succes_state_center = centerLaneMarker()
    succes_state_scale = scaleLaneMarker()

    if succes_state_center == 'success' and succes_state_scale == 'succes':
        return centerAndScale()
    
    return 'failure'


def rotateToHeading(timeout=5):
    #TEMPORARY
    return "success"
    
    
    global last_object_detection
    rotationError = 9999
    rotationTolerance = 5 #in degrees
    startTime = time.time()
    laneMarkerInView = False
    success = False
    pub = rospy.Publisher('/theta_z_setpoint', Float64, queue_size=5, latch=True)
    #wait for state theta z update
    while state_theta_z == None:
        if time.time() > startTime + timeout:
            print("Could not get state theta z.")
            return 'failure'
    while not success:
        while abs(rotationError) > rotationTolerance:
            if len(last_object_detection) > 0:
                #we assume that the angle pointing "downwards" (i.e. abs() above 90) is the angle we are coming from
                #so our target angle is the one closest to 0 degrees (most forward angle)
                rotationError = min(last_object_detection[4:6], key=lambda x : abs(x))
                #maybe? after the first time we assume our target heading is the one closest to our current theta_z_setpoint
                #rotationError = min(last_object_detection[4:6], key=lambda x : abs(z_setpoint-(state_theta_z + x)))
                z_setpoint = state_theta_z + rotationError
                print("Detection! Updating setpoint to", z_setpoint, "- current state is", state_theta_z)
                pub.publish(Float64(state_theta_z))
                startTime = time.time()
                last_object_detection = []
            else:
                print("No detection.... Timeout in " + str((startTime + timeout) - time.time()), end='\r')
                if time.time() > startTime + timeout:
                    print("Lane marker no longer visible.")
                    pub.publish(Float64(state_theta_z))
                    return 'failure'
        print("Rotational error small! Maintaining rotation to ensure it was not a fluke.")
        #when we're here we got a small error on rotation
        success = True
        verifications = 0
        startTime = time.time()
        #ensure our rotation error stays small for 10 more detections before success
        while verifications < 10:
            if len(last_object_detection) > 0:
                startTime = time.time()
                #assume target heading is the one closest to 0 degrees
                rotationError = min(last_object_detection[4:6], key=lambda x : abs(x))
                if abs(rotationError) > rotationTolerance:
                    print("Detection! Rotational error large. Verirication failed. Restarting rotation.")
                    success = False
                    break
                else:
                    verifications += 1
                    print("Detection! Rotational error small. Verification " + str(verifications) + "/10 succeeded.")
            else:
                print("No detection.... Timeout in " + str((startTime + timeout) - time.time()), end='\r')
                if time.time() > startTime + timeout:
                    print("Lane marker no longer visible.")
                    pub.publish(Float64(state_theta_z))
                    return 'failure'
    
    print("Successfully rotated to lane marker!")
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
    # freeze state_theta_z updates when we've detected a lane marker
    # so that the rotation is somewhat synced with the image
    if len(last_object_detection) == 0:
        state_theta_z = float(msg.data)

if __name__ == '__main__':
    rospy.init_node('navigate_lane_marker')
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('find', findLaneMarker(),
                transitions={'success': 'scale', 'failure':'failure'})
        smach.StateMachine.add('center', centerAndScale(), 
                transitions={'success': 'rotate', 'failure':'failure'})
        smach.StateMachine.add('rotate', rotateToHeading(), 
                transitions={'success': 'success', 'failure':'failure'})

    last_object_detection = []
    #SET STATE_THETA_Z TO NONE WHEN TESTING WITH IMU + SETPOINTS
    #state_theta_z = None
    state_theta_z = 0
    obj_sub = rospy.Subscriber('vision/viewframe_detection', ObjectDetectionFrame, objectDetectCb)
    theta_sub = rospy.Subscriber('state_theta_z', ObjectDetectionFrame, objectDetectCb)

    res = sm.execute()
