#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

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
        self.val = depth.data 

class Theta_X(_State_Var):
    def __init__(self):
        super().__init__()

class Theta_Y(_State_Var):
    def __init__(self):
        super().__init__()

class Theta_Z(_State_Var):
    def __init__(self):
        super().__init__()
