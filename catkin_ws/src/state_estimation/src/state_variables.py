#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from auv_msgs.msg import ImuData

class _State_Var:
    def __init__(self, init_val=0.0):
        self.val = init_val

    def get(self):
        return self.val

class X(_State_Var):
    def __init__(self):
        super().__init__()

class Y(_State_Var):
    def __init__(self):
        super().__init__()

class Z(_State_Var):
    def __init__(self):
        super().__init__()
        rospy.Subscriber('depth', Float64, self.depth_sensor_cb)

    def depth_sensor_cb(self, depth):
        self.val = self.convert(depth.data)

    def convert(self, depth):
        return depth * -1

class Theta_X(_State_Var):
    def __init__(self):
        super().__init__()
        rospy.Subscriber('imu_data', ImuData, self.theta_x_cb)
    def theta_x_cb(self,data):
        self.val = data.ROLL - rospy.get_param("~theta_x_offset",0.0)

class Theta_Y(_State_Var):
    def __init__(self):
        super().__init__()
        rospy.Subscriber('imu_data', ImuData, self.theta_y_cb)
    def theta_y_cb(self,data):
        self.val = data.PITCH - rospy.get_param("~theta_y_offset",0.0)
class Theta_Z(_State_Var):
    def __init__(self):
        super().__init__()
        rospy.Subscriber('imu_data', ImuData, self.theta_z_cb)
    def theta_z_cb(self,data):
        self.val = data.YAW - rospy.get_param("~theta_z_offset",0.0)
