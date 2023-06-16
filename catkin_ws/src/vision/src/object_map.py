import numpy as np
import rospy
from auv_msgs.msg import ObjectDetectionFrame, ObjectMap
import math

#callback when a new object detection frame is published
def objectDetectCb(msg):
    try:
        addObservation(msg)
        publishMap()
    except Exception as e:
        print(str(e))

#add an object detection frame to the object map
def addObservation(msg):
    for i in range(len(msg.label)):
        if msg.x[i] == None or msg.y[i] == None or msg.z[i] == None: continue
        #find which object this detection pertains to
        obj_i = findObject([msg.label[i], msg.x[i], msg.y[i], msg.z[i]])
        #if it does not pertain to any preexisting object add it to the map
        if obj_i == -1:
            object_map.append([msg.label[i], msg.x[i], msg.y[i], msg.z[i], msg.theta_z[i], msg.extra_field[i], msg.detection_confidence[i], msg.pose_confidence[i], 1])
        else:
            #otherwise update the object map with the new observation
            updateMap(obj_i, [msg.label[i], msg.x[i], msg.y[i], msg.z[i], msg.theta_z[i], msg.extra_field[i], msg.detection_confidence[i], msg.pose_confidence[i]])

#given an observation, find the object to which it pertains to (object within a certain radius, priority to objects of same class)
def findObject(observation):
    observed_label, observed_x, observed_y, observed_z = observation
    #find all objects within sameObjectRadius of this observation in the map
    close_objs = []
    #go through every object in map and add to close_objs if close enough
    for obj_i in range(len(object_map)):
        _, current_x, current_y, current_z, _, _, _, _, _ = object_map[obj_i]
        #find distance between object in map and observation
        if dist((current_x, current_y, current_z), (observed_x, observed_y, observed_z)) < sameObjectRadius:
            close_objs.append(obj_i)
    #if there is only one object within radius return that
    if len(close_objs) == 1: return close_objs[0]
    #find objects (if any) with same label as observation in close_objs
    same_label_close_objs = []
    for obj_i in close_objs:
        if object_map[obj_i][0] == observed_label:
            same_label_close_objs.append(obj_i)
    #if there are objects with same label only consider these
    if len(same_label_close_objs) != 0: close_objs = same_label_close_objs
    #return closest object to observation
    return min(close_objs, key=lambda x : dist((object_map[x][1],object_map[x][2],object_map[x][3]), (observed_x, observed_y, observed_z)))

#NOT 100% SURE ABOUT PROBABILITIES
#update the object map using probabilities to improve estimate of object pose and label
def updateMap(obj_i, observation):
    observed_label, observed_x, observed_y, observed_z, observed_theta_z, observed_extra_field, observation_label_conf, observation_pose_conf = observation
    current_label, current_x, current_y, current_z, current_theta_z, current_extra_field, current_label_conf, current_pose_conf, num_observations = object_map[obj_i]
    #CALCULATE LABEL AND LABEL CONFIDENCE
    #if observation is same label as the object in the map
    if observed_label == current_label:
        chance_of_misclassification = (1.0-observation_label_conf)*(1.0-current_label_conf)
        new_label = current_label
        #new label confidence = average of chance of correct classification with current and observed labels
        new_label_conf = ((1.0-chance_of_misclassification) + current_label_conf + observation_label_conf)/3.0
    #if observation is same label as the object in the map
    else: #might need to change label of object in map
        # TODO: use num_observations to increase confidence in current label?
        if current_label_conf < observation_label_conf: #change label
            new_label = observed_label
            #new label confidence is average of probability that previous label was incorrect and probability that new label is correct
            new_label_conf = (observation_label_conf + (1.0-current_label_conf))/2.0
        else: #keep current label
            new_label = current_label
            #new label confidence is average of probability that observation label was incorrect and probability that current label is correct
            new_label_conf = ((1.0-observation_label_conf) + current_label_conf)/2.0
    #CALCULATE POSE AND POSE CONFIDENCE
    #keep a weighted average of the pose confidence
    new_pose_conf = (num_observations*current_pose_conf + observation_pose_conf) / (num_observations + 1)
    #WEIGHTED AVERAGE USING CONFIDENCE AND NUM. OBSERVATIONS AS WEIGHT
    # TODO: find way to avoid huge x, y, z observations from changing the predicted pose by a lot (i.e. if x=99999)
    new_x = (observation_pose_conf*observed_x + num_observations*current_pose_conf*current_x) / (current_pose_conf*num_observations + observation_pose_conf)
    new_y = (observation_pose_conf*observed_y + num_observations*current_pose_conf*current_y) / (current_pose_conf*num_observations + observation_pose_conf)
    new_z = (observation_pose_conf*observed_z + num_observations*current_pose_conf*current_z) / (current_pose_conf*num_observations + observation_pose_conf)
    #if no theta z measurement keep current theta z
    if observed_theta_z == None:
        new_theta_z = current_theta_z
    #if there was no previous theta z but observation has a theta z set theta z to observation theta z
    elif current_theta_z == None:
        new_theta_z = observed_theta_z
    #otherwise make it a weighted average of both theta z's using num observations and confidence as weight
    else:
        # TODO: since theta_z is an angle: averaging directly will not work, need to use vector sum
        # get (unit vector from current_theta_z) * current_pose_conf * num_observations
        # get (unit vector from observed_theta_z) * observation_pose_conf
        # new theta_z = sum of these -> angle
        # new_pose_conf = sum of these -> 1-e^-(length of sum vector)
        new_theta_z = (observation_pose_conf*observed_theta_z + num_observations*current_pose_conf*current_theta_z) / (current_pose_conf*num_observations + observation_pose_conf)

    #CALCULATE EXTRA FIELD WHEN APPLICABLE
    #same process as theta z for lane marker since both are angles
    if observed_label == current_label:
        if observed_extra_field == None:
            new_extra_field = current_extra_field
        elif current_extra_field == None:
            new_extra_field = observed_extra_field
        else:
            if new_label == 0: # LANE MARKER
                #WEIGHTED AVERAGE USING CONFIDENCE AND NUM. OBSERVATIONS AS WEIGHT
                # TODO: since extra_field is an angle for lane marker: averaging directly will not work, need to use vector sum
                # get (unit vector from current_theta_z) * current_pose_conf * num_observations
                # get (unit vector from observed_theta_z) * observation_pose_conf
                # new theta_z = sum of these -> angle
                # new_pose_conf = sum of these -> 1-e^-(length of sum vector)
                new_extra_field = (observation_pose_conf*observed_extra_field + current_pose_conf*num_observations*current_extra_field) / (observation_pose_conf + current_pose_conf*num_observations)
            # ADD ADDITIONAL ELIF STATEMENTS FOR OTHER LABELS
    else: # if label of observation and object were different set extra field to whichever label was kept
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

#calculate euclidian distance between two objects
def dist(obj1, obj2):
    x1, y1, z1 = obj1
    x2, y2, z2 = obj2
    return math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)
    
#publish a version of the map with only the objects with a certain number of observations
def publishMap():
    confirmedMap = [obj for obj in object_map if obj[8] > min_observations]
    map_msg = ObjectMap()
    map_msg.x = [obj[1] for obj in confirmedMap]
    map_msg.y = [obj[2] for obj in confirmedMap]
    map_msg.z = [obj[3] for obj in confirmedMap]
    map_msg.theta_z = [obj[4] for obj in confirmedMap]
    map_msg.extra_field = [obj[5] for obj in confirmedMap]
    obj_pub.publish(map_msg)


min_observations = 5
object_map = []

sameObjectRadius = 1 #in same units as state_x, y, z etc (meters i think)


if __name__ == '__main__':
    obj_sub = rospy.Subscriber('vision/viewframe_detection', ObjectDetectionFrame, objectDetectCb)
    obj_pub = rospy.Publisher('vision/object_map', ObjectMap, queue_size=1)
    rospy.spin()