#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class DisplayMission():
    def __init__(self):
        self.cur_mission = None
        self.pub_cur_mission = rospy.Publisher("/planner/display_mission", String, queue_size=1)

    def updateMission(self, mission):
        self.cur_mission = mission
        msg = String(mission)
        self.pub_cur_mission.publish(mission)