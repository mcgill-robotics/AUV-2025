#!/usr/bin/env python3

import rospy
from auv_msgs.msg import DeadReckonReport
from sbg_driver.msg import SbgEkfQuat
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


class SensorCheck:
    def __init__(self):
        self.depth_last_update = 0
        self.dvl_last_update = 0
        self.imu_last_update = 0
        self.down_cam_last_update = 0
        self.front_cam_last_update = 0
        
        self.sub_depth = rospy.Subscriber('depth', Float64, self.callback_depth)
        self.sub_dvl = rospy.Subscriber('dead_reckon_report', DeadReckonReport, self.callback_dvl)
        self.sub_imu = rospy.Subscriber('sbg/ekf_quat', SbgEkfQuat, self.callback_imu)
        self.sub_down_cam = rospy.Subscriber('vision/down_cam/image_raw', Image, self.callback_down_cam) 
        self.sub_front_cam = rospy.Subscriber('vision/front_cam/image_rgb', Image, self.callback_front_cam)

    def callback_depth(self, _):
        self.depth_last_update = rospy.get_time()

    def callback_dvl(self, _):
        self.dvl_last_update = rospy.get_time()

    def callback_imu(self, _):
        self.imu_last_update = rospy.get_time()

    def callback_down_cam(self, _):
        self.down_cam_last_update = rospy.get_time()

    def callback_front_cam(self, _):
        self.front_cam_last_update = rospy.get_time()
    
    def execute(self):
        current_time = rospy.get_time()
        while not rospy.is_shutdown():
            if current_time - self.depth_last_update > 1:
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print("!!! Depth sensor not working !!!")
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            if current_time - self.dvl_last_update > 1:
                print("!!!!!!!!!!!!!!!!!!!!!!!")
                print("!!! DVL not working !!!")
                print("!!!!!!!!!!!!!!!!!!!!!!!")
            if current_time - self.imu_last_update > 1:
                print("!!!!!!!!!!!!!!!!!!!!!!!")
                print("!!! IMU not working !!!")
                print("!!!!!!!!!!!!!!!!!!!!!!!")
            if current_time - self.down_cam_last_update > 1:
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print("!!! Down camera not working !!!")
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            if current_time - self.front_cam_last_update > 1:
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print("!!! Front camera not working !!!")
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            current_time = rospy.get_time()
            

if __name__ == '__main__':
    rospy.init_node('sensors_check', log_level=rospy.DEBUG)
    check = SensorCheck()
    check.execute()
    rospy.spin()
    
    