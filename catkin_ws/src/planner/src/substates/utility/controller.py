#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose, Vector3, Vector3Stamped
from std_msgs.msg import Float64, Bool, Header
from auv_msgs.msg import StateAction, StateGoal, SuperimposerAction, SuperimposerGoal, StateQuaternionAction, StateQuaternionGoal
from math import hypot
from actionlib_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


# predefined bools so we don't have to write these out everytime we want to get a new goal

do_displace = Bool(True)
do_not_displace = Bool(False)


"""
Helper class for the planner. Takes in simple commands, converts them to 
goals and sends them to the control servers.
"""
class Controller:
    def __init__(self, header_time):
        print("starting controller")
        self.header_time = header_time

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer)
        self.tf_header = Header(frame_id="world_rotation")

        self.servers = []
        self.effort = 15
        self.seconds_per_meter = 1

        self.StateServer = actionlib.SimpleActionClient('state_server', StateAction)
        self.servers.append(self.StateServer)
        self.StateServer.wait_for_server()

        self.SuperimposerServer = actionlib.SimpleActionClient('superimposer_server', SuperimposerAction)
        self.servers.append(self.SuperimposerServer)
        self.SuperimposerServer.wait_for_server()

        self.StateQuaternionStateServer = actionlib.SimpleActionClient('state_quaternion_server', StateQuaternionAction)
        self.servers.append(self.StateQuaternionStateServer)
        self.StateQuaternionStateServer.wait_for_server()

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
        """
        Performs a coordinate transformation from the auv body frame
        to the world frame.
        """
        trans = self.tf_buffer.lookup_transform("world_rotation", "auv_rotation", self.header_time)
        offset_local = Vector3(lx, ly, lz)
        self.tf_header.stamp = self.header_time
        offset_local_stmp = Vector3Stamped(header=self.tf_header, vector=offset_local)
        offset_global = tf2_geometry_msgs.do_transform_vector3(offset_local_stmp, trans)
        return float(offset_global.vector.x), float(offset_global.vector.y), float(offset_global.vector.z)

    #method to easily get goal object
    def get_superimposer_goal(self,dofs):
        surge,sway,heave,roll,pitch,yaw = dofs
        goal = SuperimposerGoal()
        if surge is not None:
            goal.effort.force.x = surge
            goal.do_surge = Bool(True)
        else:
            goal.effort.force.x = 0
            goal.do_surge = Bool(False)
        if sway is not None:
            goal.effort.force.y = sway
            goal.do_sway = Bool(True)
        else:
            goal.effort.force.y = 0
            goal.do_sway = Bool(False)
        if heave is not None:
            goal.effort.force.z = heave
            goal.do_heave = Bool(True)
        else:
            goal.effort.force.z = 0
            goal.do_heave = Bool(False)
        if roll is not None:
            goal.effort.torque.x = roll
            goal.do_roll = Bool(True)
        else:
            goal.effort.torque.x = 0
            goal.do_roll = Bool(False)
        if pitch is not None:
            goal.effort.torque.y = pitch
            goal.do_pitch = Bool(True)
        else:
            goal.effort.torque.y = 0
            goal.do_pitch = Bool(False)
        if yaw is not None:
            goal.effort.torque.z = yaw
            goal.do_yaw = Bool(True)

        return goal
    
    #method to easily get goal object
    def get_state_goal(self,state,displace):
        x,y,z,theta_x,theta_y,theta_z = state
        goal = StateGoal()
        if x is not None:
            goal.position.x = x
            goal.do_x = Bool(True)
        else:
            goal.position.x = 0
            goal.do_x = Bool(False)
        if y is not None:
            goal.position.y = y
            goal.do_y = Bool(True)
        else:
            goal.position.y = 0
            goal.do_y = Bool(False)
        if z is not None:
            goal.position.z = z
            goal.do_z = Bool(True)
        else:
            goal.position.z = 0
            goal.do_z = Bool(False)
        if theta_x is not None:
            goal.rotation.x = theta_x
            goal.do_theta_x = Bool(True)
        else:
            goal.rotation.x = 0
            goal.do_theta_x = Bool(False)
        if theta_y is not None:
            goal.rotation.y = theta_y
            goal.do_theta_y = Bool(True)
        else:
            goal.rotation.y = 0
            goal.do_theta_y = Bool(False)
        if theta_z is not None:
            goal.rotation.z = theta_z
            goal.do_theta_z = Bool(True)
        else:
            goal.rotation.z = 0
            goal.do_theta_z = Bool(False)
        return goal
    
    def pose_goal(self,position,quaternion,displace):
        x,y,z = position
        w,qx,qy,qz = quaternion
        goal = StateQuaternionGoal()
        goal.displace = displace
        if x is not None:
            goal.pose.position.x = x
            goal.do_x = Bool(True)
        else:
            goal.pose.position.x = 0
            goal.do_x = Bool(False)
        if y is not None:
            goal.pose.position.y = y
            goal.do_y = Bool(True)
        else:
            goal.pose.position.y = 0
            goal.do_y = Bool(False)
        if z is not None:
            goal.pose.position.z = z
            goal.do_z = Bool(True)
        else:
            goal.pose.position.z = 0
            goal.do_z = Bool(False)
        if quaternion is not None:
            goal.pose.orientation.w = w
            goal.pose.orientation.x = qx
            goal.pose.orientation.y = qy
            goal.pose.orientation.z = qz
            goal.do_quaternion = Bool(True)
        else:
            goal.do_quaternion = Bool(False)
        return goal
    
    def quaternion_action(self,position,quaternion):
        goal = self.pose_goal(position,quaternion,do_not_displace)
        self.StateQuaternionStateServer.send_goal_and_wait(goal)


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
        goal = self.get_state_goal([None,None,None,x,y,z],do_not_displace)
        if callback == None:
            self.StateServer.send_goal_and_wait(goal)
        else:
            self.StateServer.send_goal(goal,done_cb=callback)

    def rotateYaw(self,z,callback=None):
        #if callback = None make this a blocking call
        #self.preemptCurrentAction()
        goal = self.get_state_goal([None,None,None,None,None,z],do_not_displace)
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
        goal = self.get_state_goal([x,y,z,None,None,None],do_not_displace)
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

        goal_state = self.get_state_goal([x,y,z,None,None,None],do_displace)

        self.StateServer.send_goal_and_wait(goal_state)     
        if(callback != None):
            callback()


    #rotate by this amount
    def rotateDelta(self,delta,callback=None):
        #self.preemptCurrentAction()
        #if callback = None make this a blocking call
        x,y,z = delta
        goal = self.get_state_goal([None,None,None,x,y,z],do_displace)
        if callback == None:
            self.StateServer.send_goal_and_wait(goal)
        else:
            self.StateServer.send_goal(goal,done_cb=callback)

    def rotateYawDelta(self, delta, callback=None):
        goal = self.get_state_goal([None, None, None, None, None, delta], do_displace)
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

        goal_state = self.get_state_goal([gx, gy, gz, None, None, None], do_not_displace)
        self.StateServer.send_goal_and_wait(goal_state)

        if(callback != None):
            callback()

    #set angular velocity
    def torque(self,vel):
        #self.preemptCurrentAction()
        x,y,z = vel
        goal = self.get_superimposer_goal([None,None,None,x,y,z])
        self.SuperimposerServer.send_goal(goal)

    #set thruster velocity in local space (i.e. z is always heave)
    def forceLocal(self,vel):
        x,y = vel
        #self.preemptCurrentAction()
        goal = self.get_superimposer_goal([x,y,None,None,None,None])
        self.SuperimposerServer.send_goal(goal)
    
    #stop all thrusters
    def kill(self):
        # self.preemptCurrentAction()
        goal = self.get_superimposer_goal([0,0,0,0,0,0])
        self.SuperimposerServer.send_goal(goal)

    #stay still in place
    def stop_in_place(self):
        self.preemptCurrentAction()
        goal = self.get_state_goal([self.x,self.y,self.z,self.theta_x,self.theta_y,self.theta_z],do_not_displace)
        self.StateServer.send_goal_and_wait(goal)
