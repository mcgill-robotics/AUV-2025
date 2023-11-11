#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
from auv_msgs.msg import DisplayScreen


def updateMission(msg):
    mission = msg.data

def updateDVL(msg):
    is_dvl_active = msg.data

def updateIMU(msg):
    is_imu_active = msg.data

def updateDepth(msg):
    is_depth_active = msg.data


if __name__ == "__main__":
    rospy.init_node("display_mission")

    mission = ""
    is_dvl_active = False
    is_imu_active = False
    is_depth_active = False

    sub_mission = rospy.Subscriber("/planner/display_mission", String, updateMission)
    sub_is_dvl_active = rospy.Subscriber("/sensor/dvl/is_active", Bool, updateDVL)
    sub_is_imu_active = rospy.Subscriber("/sensor/imu/is_active", Bool, updateIMU)
    sub_is_depth_active = rospy.Subscriber("/sensor/depth/is_active", Bool, updateDepth)

    pub_display = rospy.Publisher("/display_screen", DisplayScreen, queue_size=1)

    while not rospy.is_shutdown():
        display = DisplayScreen()
        display.mission = mission
        display.is_dvl_active = is_dvl_active
        display.is_imu_active = is_imu_active
        display.is_depth_active = is_depth_active
        pub_display.publish(display)

    rospy.spin()