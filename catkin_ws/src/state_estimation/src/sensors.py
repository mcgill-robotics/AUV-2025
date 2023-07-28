#!/usr/bin/env python3

import rospy
import numpy as np
import quaternion

from auv_msgs.msg import DeadReckonReport
from sbg_driver.msg import SbgEkfQuat, SbgImuData
from std_msgs.msg import Float64
from tf import transformations

RAD_PER_DEG = np.pi/180

# global frame relative to NED (North-East-Down)
Q_GLOBAL_NED = np.quaternion(0, 1, 0, 0) # inertial frame, will not change

class Sensor:
    def q_auv_global(self):
        return None

    def pos_auv_global(self, q_auv_global):
        return None


class DepthSensor(Sensor):
    def __init__(self):
        # static mount - depth_sensor relative to AUV frame
        self.mount_pos_ds_auv = np.array([0.0, 0.0, 0.0])

        # measurements 
        self.pos_ds_global = None # np.array([x, y, z])

        rospy.Subscriber("/depth", Float64, self.__depth_cb)


    def pos_auv_global(self, q_auv_global):
        # no data yet
        if self.pos_ds_global is None:
            return None

        # vector from depth_sensor to AUV expressed in global frame
        pos_auv_ds_global = -quaternion.rotate_vectors(q_auv_global, self.mount_pos_ds_auv)

        # postiion of AUV in global frame (according to depth sensor)
        pos_auv_global =  self.pos_ds_global + pos_auv_ds_global
        return pos_auv_global


    def __depth_cb(self, depth_msg):
        # position (z) of depth sensor in global frame
        self.pos_ds_global = np.array([0.0, 0.0, depth_msg.data]) # depth data being negative -> underwater 


class IMUSensor(Sensor):
    def __init__(self):
        # static mount - imu frame relative to AUV frame
        self.mount_q_imu_auv = np.quaternion(0, 0, 0, 1) # imu is rotated 180 degrees about z axis relative to AUV frame

        # measurements
        self.q_imu_global = None  # np.quaternion(w, x, y, z)
        self.w_imu = None         # np.array([x, y, z])

        rospy.Subscriber("/sbg/imu_data", SbgImuData, self.__ang_vel_cb)
        rospy.Subscriber("/sbg/ekf_quat", SbgEkfQuat, self.quat_cb)


    def q_auv_global(self):
        # no data yet
        if self.q_imu_global is None:
            return None

        # quaternion of AUV in global frame
        q_auv_global = self.q_imu_global*self.mount_q_imu_auv.inverse()
        return q_auv_global


    def w_auv(self):
        # no data yet
        if self.w_imu is None:
            return None

        # anuglar velocity vector relative to AUV frame 
        w_auv = quaternion.rotate_vectors(self.mount_q_imu_auv, self.w_imu)
        return w_auv


    def __ang_vel_cb(self, w_msg):
        # angular velocity vector relative to imu frame 
        self.w_imu = np.array([w_msg.gyro.x, w_msg.gyro.y, w_msg.gyro.z])


    def quat_cb(self, q_msg):
        # quaternion of imu in NED frame
        q_imu_ned = q_msg.quaternion
        q_imu_ned = np.quaternion(q_imu_ned.w, q_imu_ned.x, q_imu_ned.y, q_imu_ned.z)

        # quaternion of imu in global frame
        self.q_imu_global = Q_GLOBAL_NED.inverse()*q_imu_ned


class DVLSensor(Sensor):
    def __init__(self):
        # static mount - dvl frame relative to AUV frame
        self.mount_pos_dvl_auv = np.array([0.0, 0.0, 0.0])
        self.mount_q_dvl_auv = np.quaternion(0, 0.3826834, 0.9238795, 0) # RPY [deg]: (180, 0, -135) 

        # DVL measurements are with reference to its reset frame (dvlref) 
        self.pos_dvlref_global = None
        self.q_dvlref_global = None

        # measurements
        self.pos_dvl_dvlref = None # np.array([x, y, z])
        self.q_dvl_dvlref = None   # np.quaternion(w, x, y, z)

        rospy.Subscriber("/dead_reckon_report", DeadReckonReport, self.__dead_reckon_cb)


    def set_dvlref_global(self, q_auv_global, pos_auv_global):
        if self.q_dvl_dvlref is None or self.pos_dvl_dvlref is None:
            raise Exception("Trying to set dvlref however, no DVL data has come in yet")

        '''
        If parts of pos_auv_global are not defined by any other sensors
        and are arbitrary (ie. xy) they can be set to any arbitrary 
        value (ie. 0), dvlrfrom tf import transformationsef will be calculated such that at the current 
        moment the AUV is at pos_auv_global
        '''
        # quaternion of dvlref in global frame
        q_dvl_global = q_auv_global*self.mount_q_dvl_auv
        self.q_dvlref_global = q_dvl_global*self.q_dvl_dvlref.inverse()

        # position of dvlref in global frame
        # the position of the AUV is assumed to be already fully defined, dvlref must be 
        # such that the DVL reports the same pos_auv_global
        # global -> dvlref = global -> auv -> dvl -> dvlref
        pos_dvl_auv_global = quaternion.rotate_vectors(q_auv_global, self.mount_pos_dvl_auv)
        pos_dvlref_dvl_global = -quaternion.rotate_vectors(self.q_dvlref_global, self.pos_dvl_dvlref)
        self.pos_dvlref_global = pos_auv_global + pos_dvl_auv_global + pos_dvlref_dvl_global
        print("set_dvlref_global: pos_dvlref_global", self.pos_dvlref_global)


    def q_auv_global(self):
        # no data yet
        if self.q_dvl_dvlref is None:
            return None

        if self.q_dvlref_global is None:
            raise Exception("Trying to ascertain q_auv_global according to DVL however, dvlref is not known")

        # quaternion of dvl in global frame
        q_dvl_global = self.q_dvlref_global*self.q_dvl_dvlref

        # quaternion of AUV in global frame
        q_auv_global = q_dvl_global*self.mount_q_dvl_auv.inverse()
        return q_auv_global


    def pos_auv_global(self, q_auv_global):
        # no data yet
        if self.q_dvl_dvlref is None or self.pos_dvl_dvlref is None:
            return None

        if self.pos_dvlref_global is None or self.q_dvlref_global is None:
            raise Exception("Trying to ascertain pos_auv_global according to DVL however, dvlref is not known")

        # (as seen in global reference frame) vector addition:
        # global -> dvl = global -> dvlref -> dvl
        pos_dvl_dvlref_global = quaternion.rotate_vectors(self.q_dvlref_global, self.pos_dvl_dvlref)
        pos_dvl_global = self.pos_dvlref_global + pos_dvl_dvlref_global

        # dvl -> auv
        pos_auv_dvl_global = -quaternion.rotate_vectors(q_auv_global, self.mount_pos_dvl_auv)

        # global -> auv = global -> dvl -> auv
        pos_auv_global = pos_dvl_global + pos_auv_dvl_global
        return pos_auv_global


    def __dead_reckon_cb(self,dr_msg):
        # quaternion of dvl relative to dvlref 
        q_dvl_dvlref = transformations.quaternion_from_euler(dr_msg.roll*RAD_PER_DEG, dr_msg.pitch*RAD_PER_DEG, dr_msg.yaw*RAD_PER_DEG)
        self.q_dvl_dvlref = np.quaternion(q_dvl_dvlref[3], q_dvl_dvlref[0], q_dvl_dvlref[1], q_dvl_dvlref[2]) # transformations returns quaternion as nparray [x, y, z, w]

        # position of DVL relative to dvlref 
        self.pos_dvl_dvlref = np.array([dr_msg.x, dr_msg.y, dr_msg.z])
