#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose, Vector3, Vector3Stamped
from std_msgs.msg import Float64, Bool, Header
from auv_msgs.msg import StateAction, StateGoal, SuperimposerAction, SuperimposerGoal
from math import hypot
from actionlib_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

do_xyz = [Bool(True),Bool(True),Bool(True),Bool(False),Bool(False),Bool(False)]
do_xy = [Bool(True),Bool(True),Bool(False),Bool(False),Bool(False),Bool(False)]
do_z = [Bool(False),Bool(False),Bool(True),Bool(False),Bool(False),Bool(False)]
do_txtytz = [Bool(False),Bool(False),Bool(False),Bool(True),Bool(True),Bool(True)]
do_tz = [Bool(False),Bool(False),Bool(False),Bool(False),Bool(False),Bool(True)]
do_all = [Bool(True)] * 6

do_displace = Bool(True)
do_not_displace = Bool(False)

class Controller:

    def __init__(self, header_time):
        print("starting controller")
        self.header_time = header_time

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer)
        self.tf_header = Header(frame_id="world")

        self.servers = []
        self.effort = 15
        self.seconds_per_meter = 1

        self.StateServer = actionlib.SimpleActionClient('state_server', StateAction)
        self.servers.append(self.StateServer)
        self.StateServer.wait_for_server()

        self.LocalSuperimposerServer = actionlib.SimpleActionClient('superimposer_local_server', SuperimposerAction)
        self.servers.append(self.LocalSuperimposerServer)
        self.LocalSuperimposerServer.wait_for_server()

        self.GobalSuperimposerServer = actionlib.SimpleActionClient('superimposer_global_server', SuperimposerAction)
        self.servers.append(self.GobalSuperimposerServer)
        self.GobalSuperimposerServer.wait_for_server()

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
    
    def transformLocalToGlobal(self,lx,ly,lz):
        trans = self.tf_buffer.lookup_transform("world", "auv_base", self.header_time)
        offset_local = Vector3(lx, ly, lz)
        self.tf_header.stamp = self.header_time
        offset_local_stmp = Vector3Stamped(header=self.tf_header, vector=offset_local)
        offset_global = tf2_geometry_msgs.do_transform_vector3(offset_local_stmp, trans)
        return float(offset_global.vector.x), float(offset_global.vector.y), float(offset_global.vector.z)

    #method to easily get goal object
    def get_superimposer_goal(self,dofs,keepers,displace):
        surge,sway,heave,roll,pitch,yaw = dofs
        goal = SuperimposerGoal()
        goal.effort.force.x = surge
        goal.effort.force.y = sway
        goal.effort.force.z = heave
        goal.effort.torque.x = roll
        goal.effort.torque.y = pitch
        goal.effort.torque.z = yaw
        goal.displace = displace
        goal.do_surge, goal.do_sway, goal.do_heave, goal.do_roll, goal.do_pitch, goal.do_yaw = keepers
        return goal
    
    #method to easily get goal object
    def get_state_goal(self,state,keepers,displace):
        x,y,z,theta_x,theta_y,theta_z = state
        goal = StateGoal()
        goal.position.x = x
        goal.position.y = y
        goal.position.z = z
        goal.rotation.x = theta_x
        goal.rotation.y = theta_y
        goal.rotation.z = theta_z
        goal.displace = displace
        goal.do_x, goal.do_y, goal.do_z, goal.do_theta_x, goal.do_theta_y, goal.do_theta_z = keepers
        return goal

    #preempt the current action
    def preemptCurrentAction(self):
        for server in self.servers:
            if server.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                server.cancel_goal()
        pass

    #rotate to this rotation
    def rotate(self,ang,callback=None):
        #if callback = None make this a blocking call
        #self.preemptCurrentAction()
        x,y,z = ang
        goal = self.get_state_goal([0,0,0,x,y,z],do_txtytz,do_not_displace)
        if callback == None:
            self.StateServer.send_goal_and_wait(goal)
        else:
            self.StateServer.send_goal(goal,done_cb=callback)

    #REQUIRES DVL
    #move to setpoint
    def move(self,pos,callback=None):
        #self.preemptCurrentAction()
        #if callback = None make this a blocking call
        x,y,z = pos
        goal = self.get_state_goal([x,y,z,0,0,0],do_xyz,do_not_displace)
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

        dist = hypot(x,y)

        if dist == 0:
            x_effort = 0
            y_effort = 0
        else:
            x_effort = x * self.effort / dist
            y_effort = y * self.effort / dist

        time = self.seconds_per_meter * dist

        #self.preemptCurrentAction()
        #if callback = None make this a blocking call
        goal_super = self.get_superimposer_goal([x_effort,y_effort,0,0,0,0],do_xy,do_not_displace)
        goal_state = self.get_state_goal([0,0,z,0,0,0],do_z,do_displace)

        self.StateServer.send_goal_and_wait(goal_state)
        
        self.GobalSuperimposerServer.send_goal(goal_super)
        rospy.sleep(time)
        self.GobalSuperimposerServer.cancel_goal()
        if(callback != None):
            callback()


    #rotate by this amount
    def rotateDelta(self,delta,callback=None):
        #self.preemptCurrentAction()
        #if callback = None make this a blocking call
        x,y,z = delta
        goal = self.get_state_goal([0,0,0,x,y,z],do_txtytz,do_displace)
        if callback == None:
            self.StateServer.send_goal_and_wait(goal)
        else:
            self.StateServer.send_goal(goal,done_cb=callback)

    def rotateYaw(self, delta, callback=None):
        goal = self.get_state_goal([0, 0, 0, 0, 0, delta], do_tz, do_displace)
        if callback == None:
            self.StateServer.send_goal_and_wait(goal)
        else:
            self.StateServer.send_goal(goal, done_cb=callback)


    #REQUIRES DVL
    #NOTE: FOR NOW WE CAN APPROXIMATE WITH MOVING FORWARD FOR X SECONDS FOR POOL TEST
    #move by this amount in local space (i.e. z is always heave)
    def moveDeltaLocal(self,delta,callback=None):
        x,y,z = delta

        delta_gx, delta_gy, delta_gz  = self.transformLocalToGlobal(x, y, z)

        gx = delta_gx + self.x
        gy = delta_gy + self.y
        gz = delta_gz + self.z

        goal_state = self.get_state_goal([gx, gy, gz, 0, 0, 0], do_xyz, do_displace)
        self.StateServer.send_goal_and_wait(goal_state)

        self.LocalSuperimposerServer.cancel_goal()
        if(callback != None):
            callback()

    #set angular velocity
    def torque(self,vel):
        #self.preemptCurrentAction()
        x,y,z = vel
        goal = self.get_superimposer_goal([0,0,0,x,y,z],do_txtytz,do_not_displace)
        self.GobalSuperimposerServer.send_goal(goal)

    #set thruster velocity output
    def force(self,vel):
        x,y = vel
        #self.preemptCurrentAction()
        goal = self.get_superimposer_goal([x,y,0,0,0,0],do_xy,do_not_displace)
        self.GobalSuperimposerServer.send_goal(goal)

    #set thruster velocity in local space (i.e. z is always heave)
    def forceLocal(self,vel):
        x,y = vel
        #self.preemptCurrentAction()
        goal = self.get_superimposer_goal([x,y,0,0,0,0],do_xy,do_not_displace)
        self.LocalSuperimposerServer.send_goal(goal)
    
    #stop all thrusters
    def kill(self):
        self.preemptCurrentAction()
        goal = self.get_superimposer_goal([0,0,0,0,0,0],do_all,do_not_displace)
        self.LocalSuperimposerServer.send_goal(goal)

    #stay still in place
    def stop_in_place(self):
        self.preemptCurrentAction()
        goal = self.get_state_goal([self.x,self.y,self.z,self.theta_x,self.theta_y,self.theta_z],do_all,do_not_displace)
        self.StateServer.send_goal_and_wait(goal)
