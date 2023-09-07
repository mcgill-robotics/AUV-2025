#!/usr/bin/env python3

import rospy
import numpy as np
import quaternion

from auv_msgs.msg import DeadReckonReport
from sbg_driver.msg import SbgEkfQuat, SbgImuData
from std_msgs.msg import Float64
from tf import transformations

class Sensor():
    def __init__(self, sensor_name):
        self.time_before_considered_inactive = 1 #seconds
        self.sensor_name = sensor_name

        self.x = None
        self.y = None
        self.z = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.quaternion = None
        self.angular_velocity = None

        self.last_unique_state_time = rospy.get_time()
        self.last_state = [self.x,self.y,self.z,self.roll,self.pitch,self.yaw,self.quaternion,self.angular_velocity]
    
    def updateLastState(self):
        current_state = [self.x,self.y,self.z,self.roll,self.pitch,self.yaw,self.quaternion,self.angular_velocity]
        if current_state != self.last_state:
            if rospy.get_time() - self.last_unique_state_time > self.time_before_considered_inactive:
                rospy.loginfo("{} has become active.".format(sensor_name))
            self.last_unique_state_time = rospy.get_time() 
        self.last_state = current_state
        
    def isActive(self):
        if rospy.get_time() - self.last_unique_state_time > self.time_before_considered_inactive:
            rospy.logwarn("{} has been inactive for more than {} seconds.".format(sensor_name, time_before_considered_inactive))
            return False
        else:
            return True        

class DepthSensor(Sensor):
    def __init__(self):
        super().__init__("Depth Sensor")
        self.z_pos_mount_offset = 0
        rospy.Subscriber("/depth", Float64, self.depth_cb)

    def depth_cb(self, depth_msg):
        self.z = depth_msg.data + self.z_pos_mount_offset
        self.updateLastState()

class IMU(Sensor):
    def __init__(self):
        super().__init__("IMU")
        self.quat_mount_offset = np.quaternion(0, 0, 0, 1) # imu is rotated 180 degrees about z axis relative to AUV frame
        rospy.Subscriber("/sbg/imu_data", SbgImuData, self.ang_vel_cb)
        rospy.Subscriber("/sbg/ekf_quat", SbgEkfQuat, self.quat_cb)
        rospy.Subscriber("/sbg/ekf_nav", SbgEkfQuat, self.quat_cb)
        
    def ang_vel_cb(self):
        # DO STUFF HERE ()
        self.updateLastState()

    def quat_cb(self):
        # DO STUFF HERE (update quaternion)
        self.updateLastState()
    
    def pos_cb(self):
        # DO STUFF HERE (grab positions in case DVL loses ground lock)
        self.updateLastState()

    def updateXYZ(self, x, y, z):
        # UPDATE OFFSETS BETWEEN BEST ACCURACY SENSOR AND THIS SENSOR'S READING FOR EACH AXIS
        if x is not None: 
            pass
        if y is not None: 
            pass
        if z is not None: 
            pass

class DVL(Sensor):
    def __init__(self):
        super().__init__("DVL")
        self.quat_mount_offset = np.quaternion(0, 0.3826834, 0.9238795, 0) # RPY [deg]: (180, 0, -135) 
        self.pos_mount_offset = np.array([0.0, 0.0, -0.3])

        rospy.Subscriber("/dead_reckon_report", DeadReckonReport, self.dr_cb)
    
    def dr_cb(self):
        # DO STUFF HERE
        self.updateLastState()

    def updateXYZ(self, x, y, z):
        # UPDATE OFFSETS BETWEEN BEST ACCURACY SENSOR AND THIS SENSOR'S READING FOR EACH AXIS
        if x is not None: 
            pass
        if y is not None: 
            pass
        if z is not None: 
            pass