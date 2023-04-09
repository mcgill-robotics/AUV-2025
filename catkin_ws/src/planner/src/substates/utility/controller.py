#!/usr/bin/env python3

import rospy

##ANTHONY TODO WITH ACTIONS/SERVER, BLOCKING IF THERES A CALLBACK OTHERWISE NON BLOCKING

#rotate to this rotation
def rotate(ang,callback=None):
    #if callback = None make this a blocking call
    x,y,z = ang
    pass

#REQUIRES DVL
#move to setpoint
def move(pos,callback=None):
    #if callback = None make this a blocking call
    x,y,z = pos
    pass

#rotate by this amount
def rotateDelta(delta,callback=None):
    #if callback = None make this a blocking call
    x,y,z = delta
    pass

#REQUIRES DVL
#NOTE: FOR NOW WE CAN APPROXIMATE WITH MOVING FORWARD FOR X SECONDS FOR POOL TEST
#move by this amount in local space (i.e. z is always heave)
def moveDeltaLocal(delta,callback=None):
    seconds_per_meter = 1 #etc
    #do some math to figure out how much time effort should be done

    #if callback = None make this a blocking call
    x,y,z = delta
    pass

#REQUIRES DVL
#NOTE: FOR NOW WE CAN APPROXIMATE WITH MOVING FORWARD FOR X SECONDS FOR POOL TEST
#move by this amount in world space
def moveDelta(delta,callback=None):
    seconds_per_meter = 1 #etc
    #do some math to figure out how much time effort should be done

    #if callback = None make this a blocking call
    x,y,z = delta
    pass

#change delta angular velocity (velocity to add on top of velocity required to maintain state)
def deltaAngularVelocity(vel):
    x,y,z = vel
    pass

#change delta velocity (velocity to add on top of velocity required to maintain state) in world space
def deltaVelocity(vel):
    x,y,z = vel
    pass

#change delta velocity (velocity to add on top of velocity required to maintain state) in local space (i.e. z is always heave)
def deltaVelocityLocal(vel):
    x,y,z = vel
    pass

#set angular velocity
def angularVelocity(vel):
    x,y,z = vel
    pass

#set velocity in world space
def velocity(vel):
    x,y,z = vel
    pass

#set velocity in local space (i.e. z is always heave)
def velocityLocal(vel):
    x,y,z = vel
    pass

#preempt the current action
def preemptCurrentAction():
    pass

