#!/usr/bin/env python3

import rospy

##ANTHONY TODO WITH ACTIONS

#rotate to this rotation
def rotate(x,y,z,callback):
    pass

#rotate by this amount
def rotateDelta(x,y,z,callback):
    pass

#move to setpoint
def move(x,y,z,callback):
    pass

#move by this amount
def moveDelta(x,y,z,callback):
    pass

#rotate by this amount from now on (add to velocity required to maintain state) (deg/s)
def deltaAngularVelocity(x,y,z):
    pass

#move by this amount from now on (add to velocity required to maintain state) (m/s)
def deltaVelocity(x,y,z):
    pass

#rotate by this amount from now on (deg/s)
def angularVelocity(x,y,z):
    pass

#move by this amount from now on (m/s)
def velocity(x,y,z):
    pass

