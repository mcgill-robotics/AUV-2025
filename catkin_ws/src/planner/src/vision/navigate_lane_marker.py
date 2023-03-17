#!/usr/bin/env python3

import rospy
import smach
from auv_msgs.msg import ObjectDetectionFrame
from std_msgs.msg import Float64
import time, math
import os

pwd = os.path.realpath(os.path.dirname(__file__))

def log(text):
    print(text)
    with open(pwd + "/log.txt", "a+") as f: f.write(text + "\n")

#move until a lane marker is detected
class findLaneMarker(smach.State):
    def __init__(self):
        super().__init__(outcomes=['success', 'failure'])
        
    def execute(self, ud):
        log("Starting search for lane marker.")
        '''
        Moves the AUV forward until a lane marker is detected.
        If it cannot find the lane marker within the timeout, returns "failure".
        '''
        startTime = time.time()
        forward = Float64(10.0)
        stop = Float64(0.0)
        pub = rospy.Publisher("surge_offset", Float64, queue_size=50)
        timeout=60

        #surge forward until we see a lane marker or reach timeout
        pub.publish(forward)
        log("Moving forward (surge " + str(forward) + ")")
        while len(last_object_detection) < 1:
            if time.time() > startTime + timeout:
                log("Did not find lane marker in time.")
                pub.publish(stop)
                return 'failure'
        log("Lane marker in view!")
        pub.publish(stop)
        return 'success'

class centerAndScale(smach.State):
    def __init__(self):
        super().__init__(outcomes=['success', 'failure'])
        
    def execute(self, ud):
        '''
        Centers and scales the lane marker until error is within tolerance.
        Once the lane marker is centered and scale, it returns success.
        If it loses sight of the lane marker for more than timeout, returns "failure".
        '''
        log("Starting centering and scaling of lane marker.")
        global last_object_detection
        timeout = 5
        startTime = time.time()
        pubx = rospy.Publisher('/global_x', Float64, queue_size=1, latch=True)
        puby = rospy.Publisher('/global_y', Float64, queue_size=1, latch=True)
        pubz = rospy.Publisher('/z_setpoint', Float64, queue_size=1, latch=True)
        targetCenterX = 0.5
        targetCenterY = 0.5
        targetScale = 0.5
        centering_tolerance = 0.05
        scaling_tolerance = 0.05
        centered = False
        scaled = True
        z_offset_value = 1
        surge_p_value = 10
        sway_p_value = 10

        while state_z == None:
            if time.time() > startTime + timeout:
                log("Could not get state z.")
                return 'failure'
        startTime = time.time()

        while not (centered and scaled):
            if len(last_object_detection) > 0:
                center_x = last_object_detection[6]
                center_y = last_object_detection[7]
                width = last_object_detection[2]
                height = last_object_detection[3]
                centering_error = math.sqrt((center_x - targetCenterX)**2 + (center_y - targetCenterY)**2)
                scaling_error = abs(max(width, height) - targetScale)
                
                log("Lane marker in view at: x:{}, y:{}, w:{}, h:{}!".format(center_x, center_y, width, height))

                if scaling_error > scaling_tolerance:
                    offsetZ = scaling_error*z_offset_value
                    pubz.publish(Float64(state_z + offsetZ))
                    log("Moving Z setpoint by {}".format(offsetZ))
                    scaled = False
                else:
                    log("Lane marker scaled!")
                    pubz.publish(Float64(state_z))
                    scaled = True

                if centering_error > centering_tolerance: #isn't centered
                    surgeValue = (targetCenterX - center_x)*surge_p_value
                    swayValue = (targetCenterY - center_y)*sway_p_value
                    log("Surging {}".format(surgeValue))
                    log("Swaying {}".format(swayValue))
                    pubx.publish(Float64(surgeValue))
                    puby.publish(Float64(swayValue))
                    centered = False
                else:
                    log("Lane marker centered!")
                    pubx.publish(Float64(0))
                    puby.publish(Float64(0))
                    centered = True
                    
                startTime = time.time()
                last_object_detection = []
            else:
                log("No detection.... Timeout in " + str((startTime + timeout) - time.time()), end='\r')
                if time.time() > startTime + timeout:
                    log("Lane marker no longer visible.")
                    pubx.publish(Float64(0))
                    puby.publish(Float64(0))
                    pubz.publish(Float64(state_z))
                    return 'failure'
                    
        log("Successfully positioned above lane marker!")
        pubx.publish(Float64(0))
        puby.publish(Float64(0))
        pubz.publish(Float64(state_z))
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
        #return "success" #TEMPORARY UNTIL IMU MEASUREMENT WORKS MORE RELIABLY
        log("Starting rotation of AUV to target heading.")
        timeout = 5
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
                log("Could not get state theta z.")
                return 'failure'
        startTime = time.time()
        while not success:
            while abs(rotationError) > rotationTolerance:
                if len(last_object_detection) > 0:
                    #we assume that the angle pointing most "downwards" (i.e. highest abs() angle) is the angle we are coming from
                    #so our target angle is the one closest to 0 degrees (most forward angle)
                    #since we make our target heading get closer to 0 we can choose the most forward angle at every iteration
                    rotationError = min(last_object_detection[4:6], key=lambda x : abs(x))
                    #maybe? after the first time we assume our target heading is the one closest to our current theta_z_setpoint
                    #rotationError = min(last_object_detection[4:6], key=lambda x : abs(theta_z_setpoint-(state_theta_z + x)))
                    theta_z_setpoint = state_theta_z + rotationError
                    log("Detection! Updating setpoint to", theta_z_setpoint, "- current state is", state_theta_z, "(relative heading is", str(rotationError) + ")")
                    pub.publish(Float64(theta_z_setpoint))
                    startTime = time.time()
                    last_object_detection = []
                else:
                    log("No detection.... Timeout in " + str((startTime + timeout) - time.time()), end='\r')
                    if time.time() > startTime + timeout:
                        log("Lane marker no longer visible.")
                        pub.publish(Float64(state_theta_z))
                        return 'failure'
            log("Rotational error small! Maintaining rotation to ensure it was not a fluke.")
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
                        log("Detection! Rotational error large. Verirication failed. Restarting rotation.")
                        success = False
                        break
                    else:
                        verifications += 1
                        log("Detection! Rotational error small. Verification " + str(verifications) + "/10 succeeded.")
                else:
                    log("No detection.... Timeout in " + str((startTime + timeout) - time.time()), end='\r')
                    if time.time() > startTime + timeout:
                        log("Lane marker no longer visible.")
                        pub.publish(Float64(state_theta_z))
                        return 'failure'
        
        log("Successfully rotated to lane marker!")
        pub.publish(Float64(state_theta_z))
        return 'success'


class descend(smach.State):
    def __init__(self):
        super().__init__(outcomes=['success'])
        
    def execute(self, ud):
        pub = rospy.Publisher("/z_setpoint", Float64, queue_size=50)
        pub.publish(Float64(-2.0)) 
        log("Descending AUV 2.0 meters underwater. Waiting 10 secs.")
        rospy.sleep(10)
        return 'done'
        
   
class ascendWithFailure(smach.State):
    def __init__(self):
        super().__init__(outcomes=['success'])
        
    def execute(self, ud):
        pub = rospy.Publisher("/z_setpoint", Float64, queue_size=50)
        pub.publish(Float64(0)) 
        log("Ascending AUV. Waiting 10 secs.")
        rospy.sleep(10)
        return 'success'
        
    
class ascendWithSuccess(smach.State):
    def __init__(self):
        super().__init__(outcomes=['success'])
        
    def execute(self, ud):
        pub = rospy.Publisher("/z_setpoint", Float64, queue_size=50)
        pub.publish(Float64(0)) 
        log("Ascending AUV. Waiting 10 secs.")
        rospy.sleep(10)
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
    log(str(detection))
    return detection

def objectDetectCb(msg):
    global last_object_detection
    global firstMessage
    firstMessage = True
    try:
        last_object_detection = parseMessage(msg)
    except Exception as e:
        log(str(e))

def updateTheta(msg):
    global state_theta_z
    # freeze state_theta_z updates when we've detected a lane marker
    # so that the rotation is somewhat synced with the image
    if len(last_object_detection) == 0:
        state_theta_z = float(msg.data)

def updateZPos(msg):
    global state_z
    # freeze state_theta_z updates when we've detected a lane marker
    # so that the rotation is somewhat synced with the image
    state_z = float(msg.data)

def shutdown():
    pub_thetaz = rospy.Publisher('/theta_z_setpoint', Float64, queue_size=5, latch=True)
    pub_z = rospy.Publisher("/z_setpoint", Float64, queue_size=50)
    pug_global_x = rospy.Publisher('/global_x', Float64, queue_size=1, latch=True)
    pug_global_y = rospy.Publisher('/global_y', Float64, queue_size=1, latch=True)
    if state_theta_z is None:
        pub_thetaz.publish(Float64(0))
    else:
        pub_thetaz.publish(Float64(state_theta_z))
    pub_z.publish(Float64(0))
    pug_global_x.publish(Float64(0))
    pug_global_y.publish(Float64(0))


if __name__ == '__main__':
    rospy.init_node('navigate_lane_marker')
    rospy.on_shutdown(shutdown)
    last_object_detection = []
    firstMessage = False
    #state_theta_z = None
    state_theta_z = 0 #TEMPORARY UNTIL IMU WORKS
    state_z = None
    obj_sub = rospy.Subscriber('vision/viewframe_detection', ObjectDetectionFrame, objectDetectCb)
    theta_sub = rospy.Subscriber('state_theta_z', Float64, updateTheta)
    z_pos_sub = rospy.Subscriber('state_z', Float64, state_z)

    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('descend', descend(),
                transitions={'success': 'find'})
        smach.StateMachine.add('find', findLaneMarker(),
                transitions={'success': 'scale_and_center', 'failure':'ascend_fail'})
        smach.StateMachine.add('scale_and_center', centerAndScale(), 
                transitions={'success': 'rotate', 'failure':'ascend_fail'})
        smach.StateMachine.add('rotate', rotateToHeading(), 
                transitions={'success': 'ascend_success', 'failure':'ascend_fail'})
        smach.StateMachine.add('ascend_fail', ascendWithFailure(),
                transitions={'success': 'failure'})
        smach.StateMachine.add('ascend_success', ascendWithSuccess(),
                transitions={'success': 'success'})
    while not firstMessage: pass
    log("Received first detection message. Starting state machine.")
    res = sm.execute()
    log("Ending state machine.")
