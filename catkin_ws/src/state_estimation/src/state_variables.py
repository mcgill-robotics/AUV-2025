#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from auv_msgs.msg import ImuData

class _State_Var:
    def __init__(self, init_val=0.0):
        self.val = init_val
        self.previousVal = None #start at none in case imu data starts out very different from init_val
        self.valModuloOffset = 0
        self.valChangeTol = 90 #might have to modify depending how imu modulo works
        self.moduloAmt = 360 #might be 180 depending how imu modulo works

    def get(self):
        return self.val + self.valModuloOffset

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
        rospy.Subscriber('imu_data', ImuData, self.theta_x_cb, queue_size=1)
    def theta_x_cb(self,data):
        self.val = data.ROLL - rospy.get_param("theta_x_offset")
        if self.previousVal == None:
            self.previousVal = self.val
            return
        if (self.previousVal - self.val) > self.valChangeTol: #assume large changes mean modulo came into effect
            if self.val < self.previousVal: self.valModuloOffset += self.moduloAmt
            else: self.valModuloOffset -= self.moduloAmt
        self.previousVal = self.val

class Theta_Y(_State_Var):
    def __init__(self):
        super().__init__()
        rospy.Subscriber('imu_data', ImuData, self.theta_y_cb, queue_size=1)
    def theta_y_cb(self,data):
        self.val = data.PITCH - rospy.get_param("theta_y_offset")
        if self.previousVal == None:
            self.previousVal = self.val
            return
        if (self.previousVal - self.val) > self.valChangeTol: #assume large changes mean modulo came into effect
            if self.val < self.previousVal: self.valModuloOffset += self.moduloAmt
            else: self.valModuloOffset -= self.moduloAmt
        self.previousVal = self.val

class Theta_Z(_State_Var):
    def __init__(self):
        super().__init__()
        rospy.Subscriber('imu_data', ImuData, self.theta_z_cb, queue_size=1)
    def theta_z_cb(self,data):
        self.val = data.YAW - rospy.get_param("theta_z_offset")
        if self.previousVal == None:
            self.previousVal = self.val
            return
        if (self.previousVal - self.val) > self.valChangeTol: #assume large changes mean modulo came into effect
            if self.val < self.previousVal: self.valModuloOffset += self.moduloAmt
            else: self.valModuloOffset -= self.moduloAmt
        self.previousVal = self.val
