import numpy as np
import rospy
from auv_msgs.msg import ObjectDetectionFrame
import math

def objectDetectCb(msg):
    try:
        addObservation(msg)
    except Exception as e:
        log(str(e))

def addObservation(msg):
    for i in range(len(msg.label)):
        if msg.obj_x[i] == None or msg.obj_y[i] == None or msg.obj_z[i] == None: continue
        obj_i = findObject([msg.label[i], msg.obj_x[i], msg.obj_y[i], msg.obj_z[i]])
        if obj_i == -1:
            object_map.append([msg.label[i], msg.obj_x[i], msg.obj_y[i], msg.obj_z[i], msg.obj_theta_z[i], msg.extra_field[i], msg.detection_confidence[i], msg.pose_confidence[i], 1])
        else:
            updateMap(obj_i, [msg.label[i], msg.obj_x[i], msg.obj_y[i], msg.obj_z[i], msg.obj_theta_z[i], msg.extra_field[i], msg.detection_confidence[i], msg.pose_confidence[i]])

def findObject(observation):
    observed_label, observed_x, observed_y, observed_z = observation
    #return -1 if does not correspond to an object already in the map
    #return index of object it corresponds to in the map otherwise
    close_objs = []
    for obj_i in range(len(object_map)):
        _, current_x, current_y, current_z, _, _, _, _, _ = object_map[obj_i]
        if dist((current_x, current_y, current_z), (observed_x, observed_y, observed_z)) < sameObjectRadius:
            close_objs.append(obj_i)
    #if there is only one object within radius return that
    if len(close_objs) == 1: return close_objs[0]
    #find objects (if any) with same label as observation
    same_label_close_objs = []
    for obj_i in close_objs:
        if object_map[obj_i][0] == observed_label:
            same_label_close_objs.append(obj_i)
    #if there are objects with same label only consider these
    if len(same_label_close_objs) != 0: close_objs = same_label_close_objs
    #return closest object left
    return min(close_objs, key=lambda x : dist((object_map[x][1],object_map[x][2],object_map[x][3]), (observed_x, observed_y, observed_z)))

#NOT 100% SURE ABOUT PROBABILITIES
def updateMap(obj_i, observation):
    observed_label, observed_x, observed_y, observed_z, observed_theta_z, observed_extra_field, observation_label_conf, observation_pose_conf = observation
    current_label, current_x, current_y, current_z, current_theta_z, current_extra_field, current_label_conf, current_pose_conf, num_observations = object_map[obj_i]
    #CALCULATE LABEL AND LABEL CONFIDENCE
    if observed_label == current_label:
        chance_of_misclassification = (1.0-observation_label_conf)*(1.0-current_label_conf)
        new_label = current_label
        #average chance of correct classification with current and observed labels
        new_label_conf = ((1.0-chance_of_misclassification) + current_label_conf + observation_label_conf)/3.0
    else: #might need to update label
        # use num_observations?
        if current_label_conf < observation_label_conf: #change label
            new_label = observed_label
            new_label_conf = (observation_label_conf + (1.0-current_label_conf))/2.0
        else: #keep current label
            new_label = current_label
            new_label_conf = ((1.0-observation_label_conf) + current_label_conf)/2.0
    #CALCULATE POSE AND POSE CONFIDENCE
    #MAINTAIN AVERAGE OF POSE CONFIDENCE
    new_pose_conf = (num_observations*current_pose_conf + observation_pose_conf) / (num_observations + 1)
    #WEIGHTED AVERAGE USING CONFIDENCE AND NUM. OBSERVATIONS AS WEIGHT
    new_x = (observation_pose_conf*observed_x + num_observations*current_pose_conf*current_x) / (current_pose_conf*num_observations + observation_pose_conf)
    new_y = (observation_pose_conf*observed_y + num_observations*current_pose_conf*current_y) / (current_pose_conf*num_observations + observation_pose_conf)
    new_z = (observation_pose_conf*observed_z + num_observations*current_pose_conf*current_z) / (current_pose_conf*num_observations + observation_pose_conf)
    if observed_theta_z == None:
        new_theta_z = current_theta_z
    elif current_theta_z == None:
        new_theta_z = observed_theta_z
    else:
        new_theta_z = (observation_pose_conf*observed_theta_z + num_observations*current_pose_conf*current_theta_z) / (current_pose_conf*num_observations + observation_pose_conf)

    #CALCULATE EXTRA FIELD WHEN APPLICABLE
    if observed_label == current_label:
        if observed_extra_field == None:
            new_extra_field = current_extra_field
        elif current_extra_field == None:
            new_extra_field = observed_extra_field
        else:
            if new_label == 0: # LANE MARKER
                #WEIGHTED AVERAGE USING CONFIDENCE AND NUM. OBSERVATIONS AS WEIGHT
                new_extra_field = (observation_pose_conf*observed_extra_field + current_pose_conf*num_observations*current_extra_field) / (observation_pose_conf + current_pose_conf*num_observations)
            # ADD ADDITIONAL ELIF STATEMENTS FOR OTHER LABELS
    else:
        if observed_label == new_label:
            new_extra_field = observed_extra_field
        else:
            new_extra_field = current_extra_field

    object_map[obj_i][0] = new_label
    object_map[obj_i][1] = new_x
    object_map[obj_i][2] = new_y
    object_map[obj_i][3] = new_z
    object_map[obj_i][4] = new_theta_z
    object_map[obj_i][5] = new_extra_field
    object_map[obj_i][6] = new_label_conf
    object_map[obj_i][7] = new_pose_conf
    if observed_label == current_label: object_map[obj_i][8] += 1
    else: object_map[obj_i][8] = 1
    

def dist(obj1, obj2):
    x1, y1, z1 = obj1
    x2, y2, z2 = obj2
    return math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)
    
max_lane_markers = 2
max_gates = 1
max_buoys = 1
max_torpedo_target = 1
max_bins = 2
max_octagon = 1

sameObjectRadius = 1 #in same units as state_x, y, z etc


if __name__ == '__main__':
    object_map = []
    obj_sub = rospy.Subscriber('vision/viewframe_detection', ObjectDetectionFrame, objectDetectCb)