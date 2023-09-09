#!/usr/bin/env python3
import rospy
from auv_msgs.msg import VisionObject, VisionObjectArray
import math

#callback when a new object detection frame is published
def objectDetectCb(msg):
    try:
        addObservation(msg)
        reduceMap()
        publishMap()
    except Exception as e:
        print(str(e))

#add an object detection frame to the object map
def addObservation(msg):

    # Loop over every object in the DetectionFrame array
    for detectionFrame in msg.detectionObjects:
        #find which object this detection pertains to
        obj_i = findClosestObject([detectionFrame.label, detectionFrame.x, detectionFrame.y, detectionFrame.z])
        #if it does not pertain to any preexisting object add it to the map
        if obj_i == -1:
            object_map.append([detectionFrame.label, detectionFrame.x, detectionFrame.y, detectionFrame.z, 
                               detectionFrame.theta_z, detectionFrame.extra_field, 1])
        else:
            #otherwise update the object map with the new observation
            updateMap(obj_i, [detectionFrame.label, detectionFrame.x, detectionFrame.y, detectionFrame.z, 
                               detectionFrame.theta_z, detectionFrame.extra_field, 1])

    

#given an observation, find the object to which it pertains to (object within a certain radius of same class)
def findClosestObject(observation, indexToIgnore=-1):
    observed_label, observed_x, observed_y, observed_z = observation
    #find all objects within sameObjectRadius of this observation in the map
    close_objs = []
    #go through every object in map and add to close_objs if close enough
    for obj_i in range(len(object_map)):
        obj_label, obj_x, obj_y, obj_z, _, _, _ = object_map[obj_i]
        if observed_label != obj_label or obj_i == indexToIgnore: continue
        #find distance between object in map and observation
        #ignore Z position when reducing map
        objs_distance_apart = dist((obj_x, obj_y, obj_z), (observed_x, observed_y, observed_z)) if indexToIgnore == -1 else dist((obj_x, obj_y, 0), (observed_x, observed_y, 0))
        if objs_distance_apart < sameObjectRadiusPerLabel.get(observed_label, 1000):
            close_objs.append(obj_i)
    #if there is only one object within radius return that
    if len(close_objs) == 0: return -1
    #return closest object to observation
    else: return min(close_objs, key=lambda i : dist((object_map[i][1],object_map[i][2],object_map[i][3]), (observed_x, observed_y, observed_z)))

def angleDifference(ang1, ang2):
    diff = (ang1-ang2) % 360
    if diff > 180: diff -= 360
    return abs(diff)

#NOT 100% SURE ABOUT PROBABILITIES
#update the object map using probabilities to improve estimate of object pose and label
def updateMap(obj_i, observation):
    _, observed_x, observed_y, observed_z, observed_theta_z, observed_extra_field, num_new_observations = observation
    label, current_x, current_y, current_z, current_theta_z, current_extra_field, num_observations = object_map[obj_i]
    
    #CALCULATE POSE
    new_x = (num_new_observations*observed_x + num_observations*current_x) / (num_observations + num_new_observations)
    new_y = (num_new_observations*observed_y + num_observations*current_y) / (num_observations + num_new_observations)
    new_z = (num_new_observations*observed_z + num_observations*current_z) / (num_observations + num_new_observations)

    if label == "Lane Marker": #LANE MARKER HAD TO DEALT WITH DIFFERENTLY FOR THETA Z AND EXTRA_FIELD
        #if no theta z measurement keep current theta z
        if observed_theta_z == -1234.5 and observed_extra_field == -1234.5:
            new_theta_z = current_theta_z
            new_extra_field = current_extra_field
        #if there was no previous theta z but observation has a theta z set theta z to observation theta z
        elif current_extra_field == -1234.5 and current_theta_z == -1234.5:
            new_theta_z = observed_theta_z
            new_extra_field = observed_extra_field
        else:
            #find closest match between observed angles and current angles
            if angleDifference(observed_theta_z, current_theta_z) < angleDifference(observed_theta_z, current_extra_field):
                #CLOSEST ANGLES ARE OBSERVED THETA_Z AND CURRENT THETA_Z
                while abs(observed_theta_z-current_theta_z) > 180:
                    if observed_theta_z > current_theta_z: observed_theta_z -= 360
                    else: observed_theta_z += 360
                new_theta_z = (num_new_observations*observed_theta_z + num_observations*current_theta_z) / (num_observations + num_new_observations)
                #CLOSEST ANGLES ARE OBSERVED EXTRA_FIELD AND CURRENT EXTRA_FIELD
                while abs(observed_extra_field-current_extra_field) > 180:
                    if observed_extra_field > current_extra_field: current_extra_field += 360
                    else: observed_extra_field += 360
                new_extra_field = (num_new_observations*observed_extra_field + num_observations*current_extra_field) / (num_observations + num_new_observations)
            else:
                #CLOSEST ANGLES ARE OBSERVED THETA_Z AND CURRENT EXTRA_FIELD
                while abs(observed_theta_z-current_extra_field) > 180:
                    if observed_theta_z > current_extra_field: observed_theta_z -= 360
                    else: observed_theta_z += 360
                new_extra_field = (num_new_observations*observed_theta_z + num_observations*current_extra_field) / (num_observations + num_new_observations)
                #CLOSEST ANGLES ARE OBSERVED EXTRA_FIELD AND CURRENT THETA_Z
                while abs(observed_extra_field-current_theta_z) > 180:
                    if observed_extra_field > current_theta_z: observed_extra_field -= 360
                    else: observed_extra_field += 360
                new_theta_z = (num_new_observations*observed_extra_field + num_observations*current_theta_z) / (num_observations + num_new_observations)
    
    else:
        #CALCULATE THETA Z
        #if no theta z measurement keep current theta z
        if observed_theta_z == -1234.5:
            new_theta_z = current_theta_z
        #if there was no previous theta z but observation has a theta z set theta z to observation theta z
        elif current_theta_z == -1234.5:
            new_theta_z = observed_theta_z
        else:
            while abs(observed_theta_z-current_theta_z) > 180:
                if observed_theta_z > current_theta_z: current_theta_z += 360
                else: observed_theta_z += 360
            #average both orientations
            new_theta_z = (num_new_observations*observed_theta_z + num_observations*current_theta_z) / (num_observations + num_new_observations)

        #CALCULATE EXTRA FIELD WHEN APPLICABLE
        if label == "Gate": #GATE, symbol on left (0 or 1) -> take weighted average
            if observed_extra_field == -1234.5:
                new_extra_field = current_extra_field
            elif current_extra_field == -1234.5:
                new_extra_field = observed_extra_field
            else:
                new_extra_field = (num_new_observations*observed_extra_field + num_observations*current_extra_field) / (num_observations + num_new_observations)

    object_map[obj_i][1] = new_x
    object_map[obj_i][2] = new_y
    object_map[obj_i][3] = new_z
    object_map[obj_i][4] = new_theta_z
    object_map[obj_i][5] = new_extra_field
    object_map[obj_i][6] += num_new_observations

#calculate euclidian distance between two objects
def dist(obj1, obj2):
    x1, y1, z1 = obj1
    x2, y2, z2 = obj2
    return math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)
    
#combine similar/close objects in the map into one object
def reduceMap():
    num_objs_deleted = 0
    for i in range(len(object_map)):
        idx = i - num_objs_deleted
        observed_label, observed_x, observed_y, observed_z, _, _, _ = object_map[idx]
        closest_obj = findClosestObject([observed_label, observed_x, observed_y, observed_z], indexToIgnore=idx)
        if closest_obj == -1: continue
        else:
            updateMap(closest_obj, object_map[idx])
            del object_map[i]
            num_objs_deleted += 1

#publish a version of the map with only the objects with a certain number of observations
def publishMap():

    confirmedMap = [obj for obj in object_map if obj[6] > min_observations]

    #Create an array of ObjectMap 
    map_msg_array = VisionObjectArray()
    for obj in confirmedMap:

        map_msg = VisionObject()
        map_msg.label = obj[0]
        map_msg.x = obj[1] 
        map_msg.y = obj[2] 
        map_msg.z = obj[3]
        map_msg.theta_z = obj[4]
        map_msg.extra_field = obj[5]

        map_msg_array.array.append(map_msg)


    obj_pub.publish(map_msg_array)


min_observations = 5
object_map = []

sameObjectRadiusPerLabel = { "Lane Marker": 3, "Earth Symbol":0.25, "Abydos Symbol": 0.25 }  #in same units as state_x, y, z etc (only for objects which can appear more than once)

rospy.init_node('object_map')
obj_sub = rospy.Subscriber('vision/viewframe_detection', VisionObjectArray, objectDetectCb)
obj_pub = rospy.Publisher('vision/object_map', VisionObjectArray, queue_size=1)
rospy.spin()