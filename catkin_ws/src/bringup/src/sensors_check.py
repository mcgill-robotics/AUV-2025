#!/usr/bin/env python3

import rospy
from auv_msgs.msg import DeadReckonReport
from sbg_driver.msg import SbgEkfQuat
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


class SensorCheck:
    def __init__(self):
        self.depth_last_update = rospy.get_time()
        self.dvl_last_update = rospy.get_time()
        self.imu_last_update = rospy.get_time()
        self.down_cam_last_update = rospy.get_time()
        self.front_cam_last_update = rospy.get_time()
        
        self.depth_last_value = [None, None, None, None, None]
        self.dvl_last_value = [None, None, None, None, None]
        self.imu_last_value = [None, None, None, None, None]
        self.down_cam_last_value = [None, None, None, None, None]
        self.front_cam_last_value = [None, None, None, None, None]
        
        self.sub_depth = rospy.Subscriber('depth', Float64, self.callback_depth)
        self.sub_dvl = rospy.Subscriber('dead_reckon_report', DeadReckonReport, self.callback_dvl)
        self.sub_imu = rospy.Subscriber('sbg/ekf_quat', SbgEkfQuat, self.callback_imu)
        self.sub_down_cam = rospy.Subscriber('vision/down_cam/image_raw', Image, self.callback_down_cam) 
        self.sub_front_cam = rospy.Subscriber('vision/front_cam/image_rgb', Image, self.callback_front_cam)

    def callback_depth(self, msg):
        self.depth_last_update = rospy.get_time()
        self.depth_last_value.append(msg) # make sure that the msg doesn't include some unique id or timestamp
        self.depth_last_value = self.depth_last_value[1:]

    def callback_dvl(self, msg):
        self.dvl_last_update = rospy.get_time()
        self.dvl_last_value.append(msg.data)
        self.dvl_last_value = self.dvl_last_value[1:]

    def callback_imu(self, msg):
        self.imu_last_update = rospy.get_time()
        self.imu_last_value.append(msg.quaternion)
        self.imu_last_value = self.imu_last_value[1:]

    def callback_down_cam(self, msg):
        self.down_cam_last_update = rospy.get_time()
        self.down_cam_last_value.append(msg.data)
        self.down_cam_last_value = self.down_cam_last_value[1:]

    def callback_front_cam(self, msg):
        self.front_cam_last_update = rospy.get_time()
        self.front_cam_last_value.append(msg.data)
        self.front_cam_last_value = self.front_cam_last_value[1:]
    
    def execute(self):
        """ Check if the sensors are working properly:
            - if sensor is not updated for more than 1 second, print error
            - if sensor sends the same value 5 times, print warning """
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            
            if current_time - self.depth_last_update > 1:
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print("!!! ERROR: Depth sensor not working !!!")
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            elif same_values(self.depth_last_value):
                print("\n##### WARNING: Depth sensor is sending the same value #####\n") 
                
            if current_time - self.dvl_last_update > 1:
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print("!!! ERROR: DVL not working !!!")
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            elif same_values(self.dvl_last_value):
                print("\n##### WARNING: DVL is sending the same value #####\n")
                
            if current_time - self.imu_last_update > 1:
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print("!!! ERROR: IMU not working !!!")
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            elif same_values(self.imu_last_value):
                print("\n##### WARNING: IMU is sending the same value #####\n")
                
            if current_time - self.down_cam_last_update > 1:
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print("!!! ERROR: Down camera not working !!!")
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            elif same_values(self.down_cam_last_value):
                print("\n##### WARNING: Down camera is sending the same value #####\n")
            
            if current_time - self.front_cam_last_update > 1:
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print("!!! ERROR: Front camera not working !!!")
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            elif same_values(self.front_cam_last_value):
                print("\n##### WARNING: Front camera is sending the same value #####\n")
            
            current_time = rospy.get_time()
            
            
def same_values(arr):
    return all(element == arr[0] for element in arr)
            

if __name__ == '__main__':
    rospy.init_node('sensors_check', log_level=rospy.DEBUG)
    check = SensorCheck()
    check.execute()
    rospy.spin()
    
    