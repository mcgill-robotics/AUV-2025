#!/usr/bin/env python3

import rospy
import numpy as np
import quaternion

from auv_msgs.msg import DeadReckonReport
from sbg_driver.msg import SbgEkfQuat, SbgImuData
from std_msgs.msg import Float64
from tf import transformations

Q_NWU_NED = np.quaternion(0, 1, 0, 0)
DEG_PER_RAD = 180 / np.pi
RAD_PER_DEG = 1 / DEG_PER_RAD

class Sensor():
    def __init__(self, sensor_name):
        self.time_before_considered_inactive = 1 #seconds
        self.sensor_name = sensor_name

        self.x = 0
        self.y = 0
        self.z = 0
        self.q_nwu_auv = np.quaternion(1, 0, 0, 0)
        self.angular_velocity = np.array([0,0,0])

        # initialize a sensor as "inactive"
        self.last_unique_state_time = 0
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
            rospy.logwarn("{} has been inactive for {} seconds.".format(sensor_name, time_before_considered_inactive))
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
        rospy.Subscriber("/sbg/imu_data", SbgImuData, self.ang_vel_cb)
        rospy.Subscriber("/sbg/ekf_quat", SbgEkfQuat, self.quat_cb)

        q_imu_auv_w = rospy.get_param("~q_imu_auv_w")
        q_imu_auv_x = rospy.get_param("~q_imu_auv_x")
        q_imu_auv_y = rospy.get_param("~q_imu_auv_y")
        q_imu_auv_z = rospy.get_param("~q_imu_auv_z")

        self.q_imu_auv = np.quaternion(q_imu_auv_w, q_imu_auv_x, q_imu_auv_y, q_imu_auv_z)
        self.q_nwu_auv = None

        
    def ang_vel_cb(self, msg):
        # angular velocity vector relative to imu frame 
        ang_vel_imu = np.array([msg.gyro.x, -msg.gyro.y, -msg.gyro.z])
        # anuglar velocity vector relative to AUV frame 
        self.angular_velocity = quaternion.rotate_vectors(self.q_imu_auv, ang_vel_imu)
        self.updateLastState()

    def quat_cb(self, msg):
        
        q_ned_imu = np.quaternion(msg.quaternion.w, msg.quaternion.x, msg.quaternion.y, msg.quaternion.z)
        q_nwu_imu = Q_NWU_NED * q_ned_imu
        self.q_nwu_auv = q_nwu_imu * self.q_imu_auv

        self.updateLastState()

class DVL(Sensor):
    def __init__(self, imu):
        super().__init__("DVL")
        rospy.Subscriber("/dead_reckon_report", DeadReckonReport, self.dr_cb)
        self.quat_mount_offset = np.quaternion(0, 0.3826834, 0.9238795, 0) # RPY [deg]: (180, 0, -135) 
        self.pos_mount_offset = np.array([0.0, 0.0, -0.3])

        q_dvl_auv_w = rospy.get_param("~q_dvl_auv_w")
        q_dvl_auv_x = rospy.get_param("~q_dvl_auv_x")
        q_dvl_auv_y = rospy.get_param("~q_dvl_auv_y")
        q_dvl_auv_z = rospy.get_param("~q_dvl_auv_z")

        self.q_dvl_auv = np.quaternion(q_dvl_auv_w, q_dvl_auv_x, q_dvl_auv_y, q_dvl_auv_z)

        auv_dvl_offset_x = rospy.get_param("~auv_dvl_offset_x")
        auv_dvl_offset_y = rospy.get_param("~auv_dvl_offset_y")
        auv_dvl_offset_z = rospy.get_param("~auv_dvl_offset_z")
        self.auv_dvl_offset = np.array([auv_dvl_offset_x, auv_dvl_offset_y, auv_dvl_offset_z])

        self.imu = imu
        self.q_dvlref_nwu = None
    
    def dr_cb(self, dr_msg):
        # quaternion/position of dvl relative to dvlref 
        q_dvlref_dvl = transformations.quaternion_from_euler(dr_msg.roll*RAD_PER_DEG, dr_msg.pitch*RAD_PER_DEG, dr_msg.yaw*RAD_PER_DEG)
        q_dvlref_dvl = np.quaternion(q_dvlref_dvl[3], q_dvlref_dvl[0], q_dvlref_dvl[1], q_dvlref_dvl[2]) # transformations returns quaternion as nparray [x, y, z, w]
        q_dvlref_auv = q_dvlref_dvl * self.q_dvl_auv
        pos_dvlref = np.array([dr_msg.x, dr_msg.y, dr_msg.z])

        #update dvl ref frame using imu
        if self.imu.isActive():
            q_nwu_auv = self.imu.q_nwu_auv
            self.q_dvlref_nwu = q_dvlref_auv * q_nwu_auv.inverse() 

        if self.q_dvlref_nwu is None: return

        pos_auv = quaternion.rotate_vectors(self.q_dvlref_nwu, pos_dvlref)
        dvl_auv_offset_rotated = quaternion.rotate_vectors(self.imu.q_nwu_auv.inverse(), self.auv_dvl_offset)
        pos_auv += dvl_auv_offset_rotated
        self.x = pos_auv[0]
        self.y = pos_auv[1]
        self.z = pos_auv[2]

        self.q_nwu_auv = self.q_dvlref_nwu.inverse() * q_dvlref_auv
        self.updateLastState()
        # if self.dvl_ref_frame is None: return

        # # quaternion of AUV from DVL
        # self.quaternion = (self.dvl_ref_frame * q_dvl_dvlref) * self.quat_mount_offset.inverse()
        # # get roll, pitch, yaw
        # np_quaternion = np.array([self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w])
        # # calculate angular velocity (not super precise but will only be used when IMU is inactive)
        # dt = (rospy.get_time() - self.last_unique_state_time)
        # ang_vel_x = (roll - self.roll) * RAD_PER_DEG / dt
        # ang_vel_y = (pitch - self.pitch) * RAD_PER_DEG / dt
        # ang_vel_z = (yaw - self.yaw) * RAD_PER_DEG / dt
        # self.angular_velocity = np.array([ang_vel_x, ang_vel_y, ang_vel_z])
        # self.roll = roll
        # self.pitch = pitch
        # self.yaw = yaw

        # # position of AUV from DVL
        # mount_offset = quaternion.rotate_vectors(self.quaternion, self.pos_mount_offset)
        # position = quaternion.rotate_vectors(self.dvl_ref_frame.inverse(), pos_dvl_dvlref) + mount_offset
        # self.x, self.y, self.z = position

        # self.updateLastState()

def updateAngularVelocity(self):
    # # calculate angular velocity (not super precise but will only be used when IMU is inactive)
    roll = transformations.euler_from_quaternion(np_quaternion, 'rxyz')[0] * DEG_PER_RAD
    pitch = transformations.euler_from_quaternion(np_quaternion, 'ryxz')[0] * DEG_PER_RAD
    yaw = transformations.euler_from_quaternion(np_quaternion, 'rzyx')[0] * DEG_PER_RAD

    dt = (rospy.get_time() - self.last_unique_state_time)
    rate_of_change_euler_x = (roll - self.roll) * RAD_PER_DEG / dt
    rate_of_change_euler_y = (pitch - self.pitch) * RAD_PER_DEG / dt
    rate_of_change_euler_z = (yaw - self.yaw) * RAD_PER_DEG / dt

    wx = rate_of_change_euler_y * sin(roll) * sin(yaw) + rate_of_change_euler_x * cos(yaw)
    wy = rate_of_change_euler_y * sin(roll) * cos(yaw) - rate_of_change_euler_x * sin(yaw)
    wz = rate_of_change_euler_y * cos(roll) + rate_of_change_euler_z

    self.angular_velocity = np.array([wx, wy, wz])

    self.roll = roll
    self.pitch = pitch
    self.yaw = yaw

