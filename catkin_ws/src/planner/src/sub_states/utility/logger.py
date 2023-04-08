#!/usr/bin/env python3

import rospy

rospy.init_node('planner_log', anonymous=True)

def log(string):
    rospy.loginfo(string)

def log_warn(string):
    rospy.logwarn(string)

def log_error(string):
    rospy.logerr(string)
