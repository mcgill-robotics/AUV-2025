#!/usr/bin/env python3

import rospy

##ANTHONY TODO WITH ACTIONS

#rotate to this rotation
def rotate(ang,callback):
    x,y,z = ang
    pass

#rotate by this amount
def rotateDelta(ang,callback):
    x,y,z = ang
    pass

#move to setpoint
def move(pos,callback):
    x,y,z = pos
    pass

#move by this amount
def moveDelta(pos,callback):
    x,y,z = pos
    pass

#rotate by this amount from now on (add to velocity required to maintain state)
def deltaAngularVelocity(ang):
    x,y,z = ang
    pass

#move by this amount from now on (add to velocity required to maintain state)
def deltaVelocity(pos):
    x,y,z = pos
    pass

#rotate by this amount from now on
def angularVelocity(ang):
    x,y,z = ang
    pass

#move by this amount from now on
def velocity(pos):
    x,y,z = pos
    pass

#preempt the current action
def preemptCurrentAction():
    pass

