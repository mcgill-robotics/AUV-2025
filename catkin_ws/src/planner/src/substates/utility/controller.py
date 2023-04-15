#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose, Float64
from auv_msgs.msg import StateAction, StateGoal, SuperimposerAction, SuperimposerGoal

##ANTHONY TODO WITH ACTIONS/SERVER, BLOCKING IF THERES A CALLBACK OTHERWISE NON BLOCKING

class Controller:

    def __init__(self):
        self.goal = None

        self.StateServer = actionlib.SimpleActionClient('state_server', StateAction)
        self.StateServer.wait_for_server()
        self.SuperimposerServer = actionlib.SimpleActionClient('superimposer_server', SuperimposerAction)
        self.SuperimposerServer.wait_for_server()
        self.DisplaceServer = actionlib.SimpleActionClient('displace_server',StateAction)
        self.DisplaceServer.wait_for_server()
        self.x = None
        self.y = None
        self.z = None
        self.theta_x = None
        self.theta_y = None
        self.theta_z = None

        self.sub = rospy.Subscriber("pose",Pose,self.set_position)
        self.sub_theta_x = rospy.Subscriber("state_theta_x",Float64, self.set_theta_x)
        self.sub_theta_y = rospy.Subscriber("state_theta_y",Float64, self.set_theta_y)
        self.sub_theta_z = rospy.Subscriber("state_theta_z",Float64, self.set_theta_z)

    def set_position(self,data):
        self.x = data.position.x
        self.y = data.position.y
        self.z = data.position.z

    def set_theta_x(self,data):
        self.theta_x = data.data
    def set_theta_y(self,data):
        self.theta_y = data.data
    def set_theta_z(self,data):
        self.theta_z = data.data

    def get_superimposer_goal(self,dofs):
        surge,sway,heave,roll,pitch,yaw = dofs
        goal = SuperimposerGoal()
        goal.force.x = surge
        goal.force.y = sway
        goal.force.z = heave
        goal.torque.x = roll
        goal.torque.y = pitch
        goal.torque.z = yaw
        goal.include_zeros = False
        return goal
    
    def get_state_goal(self,state):
        x,y,z,theta_x,theta_y,theta_z = state
        goal = StateGoal()
        goal.position.x = x
        goal.position.y = y
        goal.position.z = z
        goal.rotation.x = theta_x
        goal.rotation.y = theta_y
        goal.rotation.z = theta_z
        return goal

    #rotate to this rotation
    def rotate(self,ang,callback=None):
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

