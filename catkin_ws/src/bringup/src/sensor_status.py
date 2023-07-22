#!/usr/bin/env python3

import rospy
from auv_msgs.msg import DeadReckonReport
from sbg_driver.msg import SbgEkfQuat
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


class SensorStatus:
    def __init__(self):
        self.depth_last_update = rospy.get_time()
        self.dvl_last_update = rospy.get_time()
        self.imu_last_update = rospy.get_time()
        self.down_cam_last_update = rospy.get_time()
        self.front_cam_last_update = rospy.get_time()

        self.time_until_failure_detected = 1
        self.updateRate = 5
        repeteated_values_until_failure_detected = 5

        self.depth_last_values = [None] * repeteated_values_until_failure_detected
        self.dvl_last_values = [None] * repeteated_values_until_failure_detected
        self.imu_last_values = [None] * repeteated_values_until_failure_detected
        self.down_cam_last_values = [None] * repeteated_values_until_failure_detected
        self.front_cam_last_values = [None] * repeteated_values_until_failure_detected
        
        self.sub_depth = rospy.Subscriber('depth', Float64, self.callback_depth)
        self.sub_dvl = rospy.Subscriber('dead_reckon_report', DeadReckonReport, self.callback_dvl)
        self.sub_imu = rospy.Subscriber('sbg/ekf_quat', SbgEkfQuat, self.callback_imu)
        self.sub_down_cam = rospy.Subscriber('vision/down_cam/image_raw', Image, self.callback_down_cam) 
        self.sub_front_cam = rospy.Subscriber('vision/front_cam/image_rgb', Image, self.callback_front_cam)

    def callback_depth(self, msg):
        self.depth_last_update = rospy.get_time()
        self.depth_last_values.append(msg.data) # make sure that the msg doesn't include some unique id or timestamp
        self.depth_last_values = self.depth_last_values[1:]

    def callback_dvl(self, msg):
        self.dvl_last_update = rospy.get_time()
        self.dvl_last_values.append([msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.std, msg.status])
        self.dvl_last_values = self.dvl_last_values[1:]

    def callback_imu(self, msg):
        self.imu_last_update = rospy.get_time()
        self.imu_last_values.append([msg.quaternion.w, msg.quaternion.x, msg.quaternion.y, msg.quaternion.z])
        self.imu_last_values = self.imu_last_values[1:]

    def callback_down_cam(self, msg):
        self.down_cam_last_update = rospy.get_time()
        self.down_cam_last_values.append(msg.data)
        self.down_cam_last_values = self.down_cam_last_values[1:]

    def callback_front_cam(self, msg):
        self.front_cam_last_update = rospy.get_time()
        self.front_cam_last_values.append(msg.data)
        self.front_cam_last_values = self.front_cam_last_values[1:]
    
    def execute(self):
        """ Check if the sensors are working properly:
            - if sensor is not updated for more than self.time_until_failure_detected seconds, notify with an error
            - if sensor sends the same value 5 times, notify with a warning """
        while not rospy.is_shutdown():
            rospy.sleep(1.0/self.updateRate)
            current_time = rospy.get_time()
            
            if current_time - self.depth_last_update > self.time_until_failure_detected:
                print("##### ERROR: Depth sensor topic hasn't received message in {}s".format(current_time - self.depth_last_update))
            elif same_values(self.depth_last_values):
                print("##### WARNING: Depth sensor is sending the same value") 
                
            if current_time - self.dvl_last_update > self.time_until_failure_detected:
                print("##### ERROR: DVL topic hasn't received message in {}s".format(current_time - self.dvl_last_update))
            elif same_values(self.dvl_last_values):
                print("##### WARNING: DVL is sending the same value")
                
            if current_time - self.imu_last_update > self.time_until_failure_detected:
                print("##### ERROR: IMU topic hasn't received message in {}s".format(current_time - self.imu_last_update))
            elif same_values(self.imu_last_values):
                print("##### WARNING: IMU is sending the same value")
                
            if current_time - self.down_cam_last_update > self.time_until_failure_detected:
                print("##### ERROR: Down cam topic hasn't received message in {}s".format(current_time - self.down_cam_last_update))
            elif same_values(self.down_cam_last_values):
                print("##### WARNING: Down cam is sending the same value")
            
            if current_time - self.front_cam_last_update > self.time_until_failure_detected:
                print("##### ERROR: Front cam topic hasn't received message in {}s".format(current_time - self.front_cam_last_update))
            elif same_values(self.front_cam_last_values):
                print("##### WARNING: Front cam is sending the same value")
            
            
def same_values(arr):
    return all(element == arr[0] for element in arr) and not any(element is None for element in arr)
            

if __name__ == '__main__':
    rospy.init_node('sensor_status', log_level=rospy.DEBUG)
    rospy.sleep(5) # GIVE SENSORS TIME TO START UP TO AVOID SPAMMING WITH ERROR MESSAGES AT THE BEGINNING
    check = SensorStatus()
    check.execute()   
