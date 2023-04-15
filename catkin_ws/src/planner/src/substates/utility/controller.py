#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from auv_msgs.msg import StateAction, StateGoal, SuperimposerAction, SuperimposerGoal
from math import hypot

##ANTHONY TODO WITH ACTIONS/SERVER, BLOCKING IF THERES A CALLBACK OTHERWISE NON BLOCKING

class Controller:

    def __init__(self):
        self.servers = []
        self.effort = 15
        self.seconds_per_meter = 1
        self.StateServer = actionlib.SimpleActionClient('state_server', StateAction)
        self.servers.append(self.StateServer)
        self.StateServer.wait_for_server()
        self.SuperimposerServer = actionlib.SimpleActionClient('superimposer_server', SuperimposerAction)
        self.servers.append(self.SuperimposerServer)
        self.SuperimposerServer.wait_for_server()
        self.DisplaceServer = actionlib.SimpleActionClient('displace_server',StateAction)
        self.servers.append(self.DisplaceServer)
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

    #method to easily get goal object
    def get_superimposer_goal(self,dofs,keepers):
        surge,sway,heave,roll,pitch,yaw = dofs
        goal = SuperimposerGoal()
        goal.force.x = surge
        goal.force.y = sway
        goal.force.z = heave
        goal.torque.x = roll
        goal.torque.y = pitch
        goal.torque.z = yaw

        goal.do_surge, goal.do_sway, goal.do_heave, goal.do_roll, goal.do_pitch, goal.do_yaw = keepers
        return goal
    
    #method to easily get goal object
    def get_state_goal(self,state,keepers):
        x,y,z,theta_x,theta_y,theta_z = state
        goal = StateGoal()
        goal.position.x = x
        goal.position.y = y
        goal.position.z = z
        goal.rotation.x = theta_x
        goal.rotation.y = theta_y
        goal.rotation.z = theta_z

        goal.do_surge, goal.do_sway, goal.do_heave, goal.do_roll, goal.do_pitch, goal.do_yaw = keepers
        return goal

    #preempt the current action
    def preemptCurrentAction(self):
        for server in self.servers:
            server.cancel_goal()
        pass

    #rotate to this rotation
    def rotate(self,ang,callback=None):
        #if callback = None make this a blocking call
        self.preemptCurrentAction()
        x,y,z = ang
        goal = self.get_state_goal([0,0,0,x,y,z],[False,False,False,True,True,True])
        if callback == None:
            self.StateServer.send_goal_and_wait(goal)
        else:
            self.StateServer.send_goal(goal,done_cb=callback)

    #REQUIRES DVL
    #move to setpoint
    def move(self,pos,callback=None):
        self.preemptCurrentAction()
        #if callback = None make this a blocking call
        x,y,z = pos
        goal = self.get_state_goal([x,y,z,0,0,0],[True,True,True,False,False,False])
        if callback == None:
            self.StateServer.send_goal_and_wait(goal)
        else:
            self.StateServer.send_goal(goal,done_cb=callback)

    #REQUIRES DVL
    #NOTE: FOR NOW WE CAN APPROXIMATE WITH MOVING FORWARD FOR X SECONDS FOR POOL TEST
    #move by this amount in world space
    def moveDelta(self,delta,callback=None):
        #if callback = None make this a blocking call
        x,y,z = delta

        dist = hypot([x,y])
        x_effort = x * self.effort / dist
        y_effort = y * self.effort / dist

        time = self.seconds_per_meter * dist

        self.preemptCurrentAction()
        #if callback = None make this a blocking call
        goal_super = self.get_superimposer_goal([x_effort,y_effort,0,0,0,0],[True,True,False,False,False,False])
        goal_state = self.get_state_goal([0,0,z,0,0,0],[False,False,True,False,False,False])

        self.StateServer.send_goal_and_wait(goal_state)
        self.SuperimposerServer.send_goal(goal_super)
        rospy.sleep(time)
        self.preemptCurrentAction()
        if(callback != None):
            callback()


    #rotate by this amount
    def rotateDelta(self,delta,callback=None):
        self.preemptCurrentAction()
        #if callback = None make this a blocking call
        x,y,z = delta
        goal = self.get_state_goal([0,0,0,x,y,z],[False,False,False,True,True,True])
        if callback == None:
            self.DisplaceServer.send_goal_and_wait(goal)
        else:
            self.DisplaceServer.send_goal(goal,done_cb=callback)

    #REQUIRES DVL
    #NOTE: FOR NOW WE CAN APPROXIMATE WITH MOVING FORWARD FOR X SECONDS FOR POOL TEST
    #move by this amount in local space (i.e. z is always heave)
    def moveDeltaLocal(delta,callback=None):
        
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



