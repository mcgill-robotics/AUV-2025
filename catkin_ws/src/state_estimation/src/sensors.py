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
TIME_UNTIL_INACTIVE = 1 # after 1s of no data, consider sensor is inactive # TODO - set

# TODO - instead of strat by sensor, strat by data?
class Sensor:
    def __init__(self):
        self.is_active = False
        self.last_unique_msg_time = None
        self.last_unique_msg = None
        rospy.Timer(rospy.Duration(0.1), self.check_active) #TODO - duration, needs to be shutdown?


    def q_auv_global(self):
        return None


    def pos_auv_global(self, q_auv_global):
        return None


    # TODO - handle case when sensor publisher latches on last message when sensor inactive
    def check_active(self, _): # TODO - what is second arg?
        # if inactive - remain that way
        # if active - may have become inactive due to no recent messages
        if self.is_active and rospy.get_time() - self.last_unique_msg_time > TIME_UNTIL_INACTIVE:
            self.is_active = False
            rospy.logwarn("state_estimation sensors - " + self.id + " has become inactive")
            # TODO - update frames so that using other sensors coincides with current state_estimates


class DepthSensor(Sensor):
    def __init__(self):
        super().__init__() # TODO - check syntax
        self.id = "depth sensor"

        # static mount - depth_sensor relative to AUV frame
        self.mount_pos_ds_auv = np.array([0.0, 0.0, 0.0])

        # measurements 
        self.pos_ds_global = None # np.array([x, y, z])

        rospy.Subscriber("/depth", Float64, self.__depth_cb)


    def pos_auv_global(self, q_auv_global):
        # no data yet
        if not self.is_active:
            return None

        # vector from depth_sensor to AUV expressed in global frame
        pos_auv_ds_global = -quaternion.rotate_vectors(q_auv_global, self.mount_pos_ds_auv)

        # postiion of AUV in global frame (according to depth sensor)
        pos_auv_global =  self.pos_ds_global + pos_auv_ds_global
        return pos_auv_global


    def __depth_cb(self, depth_msg):
        self.last_unique_msg_time = rospy.get_time()
        self.last_unique_msg = depth_msg

        # position (z) of depth sensor in global frame
        self.pos_ds_global = np.array([0.0, 0.0, depth_msg.data]) # depth data being negative -> underwater 

        if not self.is_active:
            # sensor became active (again)
            rospy.loginfo("state_estimation sensors - " + self.id + " has become active")

        self.is_active = True


class IMUSensor(Sensor):
    def __init__(self):
        super().__init__() # TODO - check syntax
        self.id = "imu"

        # static mount - imu frame relative to AUV frame
        self.mount_q_imu_auv = np.quaternion(0, 0, 0, 1) # imu is rotated 180 degrees about z axis relative to AUV frame

        # measurements
        self.q_imu_global = None  # np.quaternion(w, x, y, z)
        self.w_imu = None         # np.array([x, y, z])

        rospy.Subscriber("/sbg/imu_data", SbgImuData, self.__ang_vel_cb)
        rospy.Subscriber("/sbg/ekf_quat", SbgEkfQuat, self.quat_cb)

        # sensor is active when both angular velocity and quat are received
        self.is_active_w = False
        self.is_active_q = False

        self.last_unique_w_msg_time = None 
        self.last_unique_w_msg = None
        self.last_unique_q_msg_time = None 
        self.last_unique_q_msg = None


    def q_auv_global(self):
        # no data yet
        if not self.is_active_q:
            return None

        # quaternion of AUV in global frame
        q_auv_global = self.q_imu_global*self.mount_q_imu_auv.inverse()
        return q_auv_global


    def w_auv(self):
        # no data yet
        if not self.is_active_w:
            return None

        # anuglar velocity vector relative to AUV frame 
        w_auv = quaternion.rotate_vectors(self.mount_q_imu_auv, self.w_imu)
        return w_auv


    def __ang_vel_cb(self, w_msg):
        self.last_unique_w_msg_time = rospy.get_time()
        self.last_unique_w_msg = w_msg

        # angular velocity vector relative to imu frame 
        self.w_imu = np.array([w_msg.gyro.x, w_msg.gyro.y, w_msg.gyro.z])

        self.is_active_w = True


    def quat_cb(self, q_msg):
        self.last_unique_q_msg_time = rospy.get_time()
        self.last_unique_q_msg = q_msg

        # quaternion of imu in NED frame
        q_imu_ned = q_msg.quaternion
        q_imu_ned = np.quaternion(q_imu_ned.w, q_imu_ned.x, q_imu_ned.y, q_imu_ned.z)

        # quaternion of imu in global frame
        self.q_imu_global = Q_GLOBAL_NED.inverse()*q_imu_ned

        self.is_active_q = True


    # overrides super to consider activity of both w and q data
    def check_active(self, _): # TODO - what is second arg?
        # update is_active_w
        if self.is_active_w and rospy.get_time() - self.last_unique_w_msg_time > TIME_UNTIL_INACTIVE:
            self.is_active_w = False

        # update is_active_q
        if self.is_active_q and rospy.get_time() - self.last_unique_q_msg_time > TIME_UNTIL_INACTIVE:
            self.is_active_q = False
        
        # update is_active
        if not self.is_active:
            # previously inactive
            if self.is_active_w and self.is_active_q:
                self.is_active = True
                rospy.loginfo("state_estimation sensors - " + self.id + " has become active")
        else:
            # previously active
            if not self.is_active_w or not self.is_active_q:
                self.is_active = False
                rospy.logwarn("state_estimation sensors - " + self.id + " has become inactive")


class DVLSensor(Sensor):
    def __init__(self, set_dvlref_cb):
        super().__init__() # TODO - check syntax
        self.id = "dvl"
        self.set_dvlref_cb = set_dvlref_cb

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

        # sensor is active when receiving messages and dvlref set
        self.is_active_dr = False
        self.dvlref_set = False # TODO - if dvl restarts, the dvlref will be different


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

        # print (to check continuity)
        '''
        print("pos_dvlref_global", self.pos_dvlref_global)
        print("q_dvlref_global", self.q_dvlref_global)
        print("-----------------------------------------")
        '''

        self.dvlref_set = True


    def q_auv_global(self):
        # no data yet
        if not self.is_active:
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
        self.last_unique_msg_time = rospy.get_time()
        self.last_unique_msg = dr_msg


        # quaternion of dvl relative to dvlref 
        q_dvl_dvlref = transformations.quaternion_from_euler(dr_msg.roll*RAD_PER_DEG, dr_msg.pitch*RAD_PER_DEG, dr_msg.yaw*RAD_PER_DEG)
        self.q_dvl_dvlref = np.quaternion(q_dvl_dvlref[3], q_dvl_dvlref[0], q_dvl_dvlref[1], q_dvl_dvlref[2]) # transformations returns quaternion as nparray [x, y, z, w]

        # position of DVL relative to dvlref 
        self.pos_dvl_dvlref = np.array([dr_msg.x, dr_msg.y, dr_msg.z])

        # update dvlref
        self.set_dvlref_cb()

        self.is_active_dr = True


    # overrides super to be active only after dvlref is set 
    def check_active(self, _): # TODO - what is second arg?
        # update is_active_dr (and dvlref_set)
        if self.is_active_dr and rospy.get_time() - self.last_unique_msg_time > TIME_UNTIL_INACTIVE:
            self.is_active_dr = False
            self.dvlref_set = False # if dvl is lost, dvlref needs to be reset once re-connected

        # update is_active
        if not self.is_active:
            # previously inactive
            if self.is_active_dr:
                if not self.dvlref_set:
                    # set dvlref
                    self.set_dvlref_cb()
                self.is_active = True
                rospy.loginfo("state_estimation sensors - " + self.id + " has become active")
        else:
            # previously active
            if not self.is_active_dr: # dvlref is set initially and is valid for duration of dvl connection 
                self.is_active = False
                rospy.logwarn("state_estimation sensors - " + self.id + " has become inactive")

