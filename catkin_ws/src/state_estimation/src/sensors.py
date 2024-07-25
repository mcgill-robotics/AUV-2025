#!/usr/bin/env python3

import rospy
import numpy as np
import quaternion

from auv_msgs.msg import DeadReckonReport
from sbg_driver.msg import SbgEkfQuat, SbgImuData
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import Imu
from tf import transformations

Q_NWU_NED = np.quaternion(0, 1, 0, 0)
Q_IMUNOMINAL_AUV = np.quaternion(0, 1, 0, 0)
Q_DVLNOMINAL_AUV = np.quaternion(0, 1, 0, 0)
DEG_PER_RAD = 180 / np.pi
RAD_PER_DEG = 1 / DEG_PER_RAD


class Sensor:
    def __init__(self, sensor_name, is_active=0):
        self.is_active = is_active
        self.sensor_name = sensor_name

    def get_sensor_name(self):
        return self.sensor_name

    def sensor_status_cb(self, msg):
        self.is_active = msg.data

    # @abstractmethod
    def get_reading(self):
        raise NotImplementedError("Subclass must implement abstract method")

    def get_is_active(self):
        return self.is_active


# Depth Sensor class inheriting from Sensor
class DepthSensor(Sensor):
    def __init__(self):
        super().__init__("Depth Sensor")

        self.z = 0
        self.z_pos_mount_offset = rospy.get_param("z_pos_mount_offset")

        rospy.Subscriber("/sensors/depth/status", Int32, self.sensor_status_cb)
        rospy.Subscriber("/sensors/depth/z", Float64, self.depth_cb)

    def depth_cb(self, msg):
        self.z = -1.0 * msg.data + self.z_pos_mount_offset

    def get_reading(self):
        return {"z": self.z}


# IMU class inheriting from Sensor
class IMU(Sensor):
    def __init__(self):
        super().__init__("IMU")

        q_imunominal_imu_w = rospy.get_param("q_imunominal_imu_w")
        q_imunominal_imu_x = rospy.get_param("q_imunominal_imu_x")
        q_imunominal_imu_y = rospy.get_param("q_imunominal_imu_y")
        q_imunominal_imu_z = rospy.get_param("q_imunominal_imu_z")

        self.q_imunominal_imu = np.quaternion(
            q_imunominal_imu_w,
            q_imunominal_imu_x,
            q_imunominal_imu_y,
            q_imunominal_imu_z,
        )
        self.q_imu_auv = self.q_imunominal_imu.conjugate() * Q_IMUNOMINAL_AUV
        self.q_nwu_auv = np.quaternion(1, 0, 0, 0)

        self.angular_velocity = np.array([0, 0, 0])

        rospy.Subscriber("/sensors/imu/status", Int32, self.sensor_status_cb)
        rospy.Subscriber("/sensors/imu/angular_velocity", SbgImuData, self.ang_vel_cb)
        rospy.Subscriber("/sensors/imu/quaternion", SbgEkfQuat, self.quat_cb)

    def ang_vel_cb(self, msg):
        # angular velocity vector relative to imu frame
        ang_vel_imu = np.array([msg.gyro.x, msg.gyro.y, msg.gyro.z])
        # anuglar velocity vector relative to AUV frame
        temp_angular_velocity = (
            self.q_imu_auv
            * np.quaternion(0, ang_vel_imu[0], ang_vel_imu[1], ang_vel_imu[2])
            * self.q_imu_auv.conjugate()
        )
        self.angular_velocity = np.array(
            [temp_angular_velocity.x, temp_angular_velocity.y, temp_angular_velocity.z]
        )

    def quat_cb(self, msg):
        q_ned_imu = np.quaternion(
            msg.quaternion.w, msg.quaternion.x, msg.quaternion.y, msg.quaternion.z
        )
        q_nwu_imu = Q_NWU_NED * q_ned_imu
        self.q_nwu_auv = q_nwu_imu * self.q_imu_auv

    def get_reading(self):
        return {"quaternion": self.q_nwu_auv, "angular_velocity": self.angular_velocity}


# IMUFrontCamera class inheriting from Sensor
class IMUFrontCamera(Sensor):
    def __init__(self):
        super().__init__("IMU Front Camera")

        self.q_nwu_auv = np.quaternion(1, 0, 0, 0)
        self.angular_velocity = np.array([0, 0, 0])

        rospy.Subscriber(
            "/sensors/imu_front_camera/status", Int32, self.sensor_status_cb
        )
        rospy.Subscriber("/zed/zed_node/imu/data", Imu, self.front_camera_imu_cb)

    def front_camera_imu_cb(self, msg):
        self.q_nwu_auv = np.quaternion(
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        )
        self.angular_velocity = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        )

    def get_reading(self):
        return {"quaternion": self.q_nwu_auv, "angular_velocity": self.angular_velocity}


# DVL class inheriting from Sensor
class DVL(Sensor):
    def __init__(self):
        super().__init__("DVL")

        self.x = 0
        self.y = 0
        self.z = 0

        q_dvlnominal_dvl_w = rospy.get_param("q_dvlnominal_dvl_w")
        q_dvlnominal_dvl_x = rospy.get_param("q_dvlnominal_dvl_x")
        q_dvlnominal_dvl_y = rospy.get_param("q_dvlnominal_dvl_y")
        q_dvlnominal_dvl_z = rospy.get_param("q_dvlnominal_dvl_z")

        q_dvlnominal_dvl = np.quaternion(
            q_dvlnominal_dvl_w,
            q_dvlnominal_dvl_x,
            q_dvlnominal_dvl_y,
            q_dvlnominal_dvl_z,
        )

        self.q_dvl_auv = q_dvlnominal_dvl.conjugate() * Q_DVLNOMINAL_AUV

        auv_dvl_offset_x = rospy.get_param("auv_dvl_offset_x")
        auv_dvl_offset_y = rospy.get_param("auv_dvl_offset_y")
        auv_dvl_offset_z = rospy.get_param("auv_dvl_offset_z")
        self.auv_dvl_offset = np.array(
            [auv_dvl_offset_x, auv_dvl_offset_y, auv_dvl_offset_z]
        )

        self.current_imu = None
        self.q_dvlref_nwu = None

        rospy.Subscriber("/sensors/dvl/status", Int32, self.sensor_status_cb)
        rospy.Subscriber("/sensors/dvl/pose", DeadReckonReport, self.dead_reckon_cb)

    def set_imu(self, imu):
        self.current_imu = imu

    def dead_reckon_cb(self, msg):
        # quaternion/position of dvl relative to dvlref
        # q_dvlref_dvl = quaternion.from_euler_angles(msg.roll * RAD_PER_DEG, msg.pitch * RAD_PER_DEG, msg.yaw * RAD_PER_DEG)
        if self.current_imu is not None:
            q_dvlref_dvl = transformations.quaternion_from_euler(
                msg.roll * RAD_PER_DEG, msg.pitch * RAD_PER_DEG, msg.yaw * RAD_PER_DEG
            )
            q_dvlref_dvl = np.quaternion(
                q_dvlref_dvl[3], q_dvlref_dvl[0], q_dvlref_dvl[1], q_dvlref_dvl[2]
            )
            q_dvlref_auv = q_dvlref_dvl * self.q_dvl_auv
            pos_dvlref_dvl = np.array([msg.x, msg.y, msg.z])

            is_current_imu_active = self.current_imu.get_is_active()

            current_imu_q_nwu_auv = self.current_imu.q_nwu_auv

            # update dvl ref frame using imu
            if is_current_imu_active:
                q_nwu_auv = self.current_imu.q_nwu_auv
                self.q_dvlref_nwu = q_dvlref_auv * q_nwu_auv.conjugate()

            if self.q_dvlref_nwu is None:
                return

            temp_pos_auv = (
                self.q_dvlref_nwu.conjugate()
                * np.quaternion(
                    0, pos_dvlref_dvl[0], pos_dvlref_dvl[1], pos_dvlref_dvl[2]
                )
                * self.q_dvlref_nwu
            )
            pos_auv = np.array([temp_pos_auv.x, temp_pos_auv.y, temp_pos_auv.z])

            temp_dvl_auv_offset_rotated = (
                current_imu_q_nwu_auv
                * np.quaternion(
                    0,
                    self.auv_dvl_offset[0],
                    self.auv_dvl_offset[1],
                    self.auv_dvl_offset[2],
                )
                * current_imu_q_nwu_auv.conjugate()
            )

            dvl_auv_offset_rotated = np.array(
                [
                    temp_dvl_auv_offset_rotated.x,
                    temp_dvl_auv_offset_rotated.y,
                    temp_dvl_auv_offset_rotated.z,
                ]
            )
            pos_auv -= dvl_auv_offset_rotated
        else:
            pos_auv = np.array([msg.x, msg.y, msg.z])

        self.x = pos_auv[0]
        self.y = pos_auv[1]
        self.z = pos_auv[2]

    def get_reading(self):
        return {"x": self.x, "y": self.y, "z": self.z}
