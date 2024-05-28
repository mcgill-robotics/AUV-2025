#!/usr/bin/env python3

import rospy
import numpy as np
import quaternion

from auv_msgs.msg import DeadReckonReport, PingerTimeDifference
from sbg_driver.msg import SbgEkfQuat, SbgImuData
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import Image, Imu
from tf import transformations
from rosgraph_msgs.msg import Clock

Q_NWU_NED = np.quaternion(0, 1, 0, 0)
Q_IMUNOMINAL_AUV = np.quaternion(0, 1, 0, 0)
Q_DVLNOMINAL_AUV = np.quaternion(0,1,0,0)
DEG_PER_RAD = 180 / np.pi
RAD_PER_DEG = 1 / DEG_PER_RAD


class Sensor():
     def __init__(self, sensor_name):
          self.time_before_considered_inactive = 1 # seconds
          self.last_error_message_time = rospy.get_time()
          self.sensor_name = sensor_name

          # initialize a sensor as "inactive"
          self.last_unique_reading_time = -1 * self.time_before_considered_inactive
          self.current_reading = None
          self.last_reading = None
          
    
     def update_last_reading(self):
          if self.current_reading != self.last_reading:
               if rospy.get_time() - self.last_unique_reading_time > self.time_before_considered_inactive:
                    rospy.loginfo("{} has become active.".format(self.sensor_name))
               self.last_unique_reading_time = rospy.get_time()
               self.last_reading = self.current_reading
        
     def get_is_active(self):
          if rospy.get_time() == 0: return 0
          if not self.has_valid_data(): return 0 # check that reading is complete
          if rospy.get_time() - self.last_unique_reading_time > self.time_before_considered_inactive: # check that reading has changed in last N seconds
               if rospy.get_time() - self.last_error_message_time > 1:
                    self.last_error_message_time = rospy.get_time()
                    rospy.logwarn("{} has been inactive for {} seconds.".format(self.sensor_name, self.time_before_considered_inactive))
               return 0
          else:
               return 1

     # @abstractmethod
     def has_valid_data(self):
          raise NotImplementedError("Subclass must implement abstract method") 


# DepthSensor class inheriting from Sensor
class DepthSensor(Sensor):
     """ 
     self.current_reading == z
     z: float
     """
     def __init__(self):
          super().__init__("Depth Sensor")
          
          rospy.Subscriber("/sensors/depth/z", Float64, self.depth_cb)

     def depth_cb(self, msg):
          self.current_reading = msg
          self.update_last_reading()
     
     def has_valid_data(self):
          return self.current_reading is not None


# IMU class inheriting from Sensor
class IMU(Sensor):
     """ 
     self.current_reading == [quaternion, angular_velocity]
     quaternion: np.quaternion(w, x, y, z)
     angular_velocity = np.array([x, y, z])
     """
     def __init__(self):
          super().__init__("IMU")

          self.current_reading = [np.quaternion(1, 0, 0, 0), [0, 0, 0]]
          self.last_reading = [np.quaternion(1, 0, 0, 0), [0, 0, 0]]
          
          rospy.Subscriber("/sensors/imu/quaternion", SbgEkfQuat, self.quat_cb)
          rospy.Subscriber("/sensors/imu/angular_velocity", SbgImuData, self.ang_vel_cb)

     def quat_cb(self, msg):
          q = msg.quaternion
          self.current_reading[0] = np.quaternion(q.w, q.x, q.y, q.z)
          self.update_last_reading()

     def ang_vel_cb(self, msg):
          gyro = msg.gyro
          self.current_reading[1] = np.array([gyro.x, gyro.y, gyro.z])
          self.update_last_reading()

     def has_valid_data(self):
          return self.current_reading[0] is not None and self.current_reading[1] is not None

# IMUFrontCamera class inheriting from Sensor
class IMUFrontCamera(Sensor):
     """ 
     self.current_reading == [quaternion, angular_velocity]
     quaternion: np.quaternion(w, x, y, z)
     angular_velocity = np.array([x, y, z])
     """
     def __init__(self):
          super().__init__("Front Camera IMU")

          self.current_reading = [np.quaternion(1, 0, 0, 0), [0, 0, 0]]
          self.last_reading = [np.quaternion(1, 0, 0, 0), [0, 0, 0]]
          
          rospy.Subscriber("/zed/zed_node/imu/data", Imu, self.front_camera_imu_cb)
          
     def front_camera_imu_cb(self, msg):
          self.current_reading[0] = np.quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
          self.current_reading[1] = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) 
          self.update_last_reading()

     # Override parent class method
     def has_valid_data(self):
          return self.current_reading[0] is not None and self.current_reading[1] is not None


# DVL class inheriting from Sensor
class DVL(Sensor):
     """ 
     self.current_reading == [x, y, z, roll, pitch, yaw]
     x, y, z, roll, pitch, yaw: float
     """
     def __init__(self):
          super().__init__("DVL")

          self.current_reading = [None, None, None, None, None, None]
          self.last_reading = [None, None, None, None, None, None]

          rospy.Subscriber("/sensors/dvl/pose", DeadReckonReport, self.dead_reckon_cb)

     def dead_reckon_cb(self, msg):
          self.current_reading = [msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw]
          self.update_last_reading()

     def has_valid_data(self):
          return None not in self.current_reading


# FrontCamera class inheriting from Sensor
class FrontCameraImage(Sensor):
     """ 
     self.current_reading == frame_id
     frame_id: string
     """
     def __init__(self):
          super().__init__("Front Camera Image")

          rospy.Subscriber("/vision/front_cam/color/image_raw", Image, self.front_camera_cb)
         
     def front_camera_cb(self, msg):
          self.current_reading = msg.header.frame_id
          self.update_last_reading()

     def has_valid_data(self):
          return self.current_reading is not None


# DownCamera class inheriting from Sensor
class DownCamera(Sensor):
     """ 
     self.current_reading == frame_id
     frame_id: string
     """
     def __init__(self):
          super().__init__("Down Camera")

          rospy.Subscriber("/vision/down_cam/image_raw", Image, self.down_camera_cb)
     
     def down_camera_cb(self, msg):
          self.current_reading = msg.header.frame_id
          self.update_last_reading()

     def has_valid_data(self):
          return self.current_reading is not None


# Hydrophones class inheriting from Sensor
class Hydrophones(Sensor):
     def __init__(self):
          super().__init__("Hydrophones")

          self.are_pingers_active = [False, False, False, False] 
          self.current_reading = [None, None, None, None]
          self.last_reading = [None, None, None, None]

          rospy.Subscriber("/sensors/hydrophones/pinger_time_difference", PingerTimeDifference, self.hydrophones_cb)

     def hydrophones_cb(self, msg):
          self.are_pingers_active = [msg.is_pinger1_active, msg.is_pinger2_active, msg.is_pinger3_active, msg.is_pinger4_active]
          self.current_reading = [msg.dt_pinger1, msg.dt_pinger2, msg.dt_pinger3, msg.dt_pinger4]
          self.update_last_reading()

     def has_valid_data(self):
          indexes_true = [i for i, e in enumerate(self.are_pingers_active) if e]
          indexes_non_zero = [i for i, e in enumerate(self.current_reading) if e != [0, 0, 0, 0]]
          return len(indexes_true) != 0 and indexes_true == indexes_non_zero


# Actuator class inheriting from Sensor
class Actuator(Sensor):
     """
     WAITING FOR ELECTRICAL FOR:
          - topic name
          - message type
     self.current_reading = position
     status: (int (0 or 1) or bool) - not yet decided
     position: (int or float) - not yet decided
     """
     def __init__(self):
          super().__init__("Actuator")

          self.status = False

          rospy.Subscriber("/TopicName", Int32, self.actuator_cb)
     
     def actuator_cb(self, msg):
          # self.status = msg.status
          # self.current_reading = msg.position
          self.update_last_reading()
     
     def has_valid_data(self):
          return self.status and self.current_reading is not None


def update_state(msg):
     pub_depth_sensor_status.publish(depth_sensor.get_is_active())
     pub_imu_sensor_status.publish(imu.get_is_active())
     pub_imu_front_camera_sensor_status.publish(imu_front_camera.get_is_active())
     pub_dvl_sensor_status.publish(dvl.get_is_active())     
     pub_front_camera_sensor_status.publish(front_camera_image.get_is_active())
     pub_down_camera_sensor_status.publish(down_camera.get_is_active())
     pub_hydrophones_sensor_status.publish(hydrophones.get_is_active())
     pub_actuator_sensor_status.publish(Int32(0)) # (actuator.get_is_active())
     


if __name__ == "__main__":
     rospy.init_node("sensors_status")

     depth_sensor = DepthSensor()
     imu = IMU()
     imu_front_camera = IMUFrontCamera()
     dvl = DVL()
     front_camera_image = FrontCameraImage()
     down_camera = DownCamera()
     hydrophones = Hydrophones()
     actuator = Actuator()

     pub_depth_sensor_status = rospy.Publisher("/sensors/depth/status", Int32, queue_size=1)
     pub_imu_sensor_status = rospy.Publisher("/sensors/imu/status", Int32, queue_size=1)
     pub_imu_front_camera_sensor_status = rospy.Publisher("/sensors/imu_front_camera/status", Int32, queue_size=1)
     pub_dvl_sensor_status = rospy.Publisher("/sensors/dvl/status", Int32, queue_size=1)
     pub_front_camera_sensor_status = rospy.Publisher("/sensors/front_camera/status", Int32, queue_size=1)
     pub_down_camera_sensor_status = rospy.Publisher("/sensors/down_camera/status", Int32, queue_size=1)
     pub_hydrophones_sensor_status = rospy.Publisher("/sensors/hydrophones/status", Int32, queue_size=1)
     pub_actuator_sensor_status  = rospy.Publisher("/sensors/actuator/status", Int32, queue_size=1)

     update_state_on_clock = rospy.get_param("update_state_on_clock")

     if update_state_on_clock:
          rospy.Subscriber("/clock", Clock, update_state)
          last_clock_msg_s = None
          last_clock_msg_ns = None
     else:
          timer = rospy.Timer(rospy.Duration(1.0/rospy.get_param("update_rate")), update_state)
          rospy.on_shutdown(timer.shutdown)

     rospy.spin()