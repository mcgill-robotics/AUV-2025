#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose, Vector3, Vector3Stamped
from std_msgs.msg import Float64, Bool, Header
from auv_msgs.msg import SuperimposerAction, SuperimposerGoal, StateQuaternionAction, StateQuaternionGoal
from actionlib_msgs.msg import GoalStatus
from tf import transformations
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import math


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

        self.sub = rospy.Subscriber("pose",Pose,self.set_position)
        self.sub_theta_x = rospy.Subscriber("state_theta_x",Float64, self.set_theta_x)
        self.sub_theta_y = rospy.Subscriber("state_theta_y",Float64, self.set_theta_y)
        self.sub_theta_z = rospy.Subscriber("state_theta_z",Float64, self.set_theta_z)

        self.clients = []

        self.SuperimposerClient = actionlib.SimpleActionClient('superimposer_server', SuperimposerAction)
        self.clients.append(self.SuperimposerClient)
        self.SuperimposerClient.wait_for_server()

        self.StateQuaternionStateClient = actionlib.SimpleActionClient('state_quaternion_server', StateQuaternionAction)
        self.clients.append(self.StateQuaternionStateClient)
        self.StateQuaternionStateClient.wait_for_server()

        self.x = None
        self.y = None
        self.z = None
        self.theta_x = None
        self.theta_y = None
        self.theta_z = None
        self.orientation = None

    def set_position(self,data):
        self.x = data.position.x
        self.y = data.position.y
        self.z = data.position.z
        self.orientation = data.orientation
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
        trans = self.tf_buffer.lookup_transform("world", "auv_base", self.header_time)
        offset_local = Vector3(lx, ly, lz)
        self.tf_header.stamp = self.header_time
        offset_local_stmp = Vector3Stamped(header=self.tf_header, vector=offset_local)
        offset_global = tf2_geometry_msgs.do_transform_vector3(offset_local_stmp, trans)
        return float(offset_global.vector.x), float(offset_global.vector.y), float(offset_global.vector.z)

    #method to easily get goal object
    def get_superimposer_goal(self,dofs):
        surge,sway,heave,roll,pitch,yaw = dofs
        
        goal = SuperimposerGoal()
        goal.effort.force.x = 0 if surge is None else surge
        goal.do_surge = Bool(False) if surge is None else Bool(True)

        goal.effort.force.y = 0 if sway is None else sway
        goal.do_sway = Bool(False) if sway is None else Bool(True)
        
        goal.effort.force.z = 0
        goal.do_heave = Bool(False)
        
        goal.effort.torque.x = 0
        goal.do_roll = Bool(False)
        
        goal.effort.torque.y = 0
        goal.do_pitch = Bool(False)
        
        goal.effort.torque.z = 0
        goal.do_yaw = Bool(False)
        
        # if sway is not None:
        #     goal.effort.force.y = sway
        #     goal.do_sway = Bool(True)
        # else:
        #     goal.effort.force.y = 0
        #     goal.do_sway = Bool(False)
        # if heave is not None:
        #     goal.effort.force.z = heave
        #     goal.do_heave = Bool(True)
        # else:
        #     goal.effort.force.z = 0
        #     goal.do_heave = Bool(False)
        # if roll is not None:
        #     goal.effort.torque.x = roll
        #     goal.do_roll = Bool(True)
        # else:
        #     goal.effort.torque.x = 0
        #     goal.do_roll = Bool(False)
        # if pitch is not None:
        #     goal.effort.torque.y = pitch
        #     goal.do_pitch = Bool(True)
        # else:
        #     goal.effort.torque.y = 0
        #     goal.do_pitch = Bool(False)
        # if yaw is not None:
        #     goal.effort.torque.z = yaw
        #     goal.do_yaw = Bool(True)
        # else:
        #     goal.effort.torque.z = 0
        #     goal.do_yaw = Bool(False)

        return goal
    
    #method to easily get goal object
    def get_state_goal(self,state,displace):
        x,y,z,tw,tx,ty,tz = state
        goal = StateQuaternionGoal()

        goal.displace = displace
        
        goal.pose.position.x = 0 if x is None else x
        goal.do_x = Bool(False) if x is None else Bool(True)

        goal.pose.position.y = 0 if y is None else y
        goal.do_y = Bool(False) if y is None else Bool(True)

        goal.pose.position.z = 0 if z is None else z
        goal.do_z = Bool(False) if z is None else Bool(True)
        
        goal.pose.orientation.w = 1 if tw is None else tw
        goal.pose.orientation.x = 0 if tx is None else tx
        goal.pose.orientation.y = 0 if ty is None else ty
        goal.pose.orientation.z = 0 if tz is None else tz
        goal.do_quaternion = Bool(False) if tz is None else Bool(True)

        return goal

    def euler_to_quaternion(self, roll, pitch, yaw):
        q = transformations.quaternion_from_euler(math.pi*roll/180, math.pi*pitch/180, math.pi*yaw/180, 'rxyz')
        return [q[3], q[0], q[1], q[2]]

    #preempt the current action
    def preemptCurrentAction(self):
        for client in self.clients:
            if client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                client.cancel_goal()

    #rotate to this rotation (quaternion)
    def rotate(self,ang,callback=None):
        #if callback = None make this a blocking call
        w,x,y,z = ang

        goal_state = self.get_state_goal([None,None,None,w,x,y,z],do_not_displace)
        if callback is not None:
            self.StateQuaternionStateClient.send_goal(goal_state,done_cb=callback)
        else:
            self.StateQuaternionStateClient.send_goal_and_wait(goal_state)
    
    #rotate to this rotation (euler)
    def rotateEuler(self,ang,callback=None):
        x,y,z = ang
        if x is None: x = self.theta_x
        if y is None: y = self.theta_y
        if z is None: z = self.theta_z
        self.rotate(self.euler_to_quaternion(x,y,z), callback=callback)

    #move to setpoint
    def move(self,pos,callback=None):
        #if callback = None make this a blocking call
        x,y,z = pos
        goal_state = self.get_state_goal([x,y,z,None,None,None,None],do_not_displace)
        
        if(callback is not None):
            self.StateQuaternionStateClient.send_goal(goal_state, done_cb=callback)
        else:
            self.StateQuaternionStateClient.send_goal_and_wait(goal_state)

    #move by this amount in world space
    def moveDelta(self,delta,callback=None):
        #if callback = None make this a blocking call
        x,y,z = delta

        goal_state = self.get_state_goal([x,y,z,None,None,None,None],do_displace)

        if(callback is not None):
            self.StateQuaternionStateClient.send_goal(goal_state, done_cb=callback)
        else:
            self.StateQuaternionStateClient.send_goal_and_wait(goal_state)

    #rotate by this amount (quaternion)
    def rotateDelta(self,delta,callback=None):
        #if callback = None make this a blocking call
        w,x,y,z = delta
        goal_state = self.get_state_goal([None,None,None,w,x,y,z],do_displace)
        
        if(callback is not None):
            self.SuperimposerClient.send_goal(goal_state, done_cb=callback)
        else:
            self.StateQuaternionStateClient.send_goal_and_wait(goal_state)

    #rotate by this amount (euler)
    def rotateDeltaEuler(self,delta,callback=None):
        x,y,z = delta
        if x is None: x = self.theta_x
        if y is None: y = self.theta_y
        if z is None: z = self.theta_z
        self.rotateDelta(self.euler_to_quaternion(x,y,z), callback=callback)

    #move by this amount in local space (i.e. z is always heave)
    def moveDeltaLocal(self,delta,callback=None):
        x,y,z = delta
        gx, gy, gz  = self.transformLocalToGlobal(x, y, z)
        goal_state = self.get_state_goal([gx, gy, gz, None, None, None, None], do_not_displace)

        if(callback is not None):
            self.StateQuaternionStateClient.send_goal(goal_state, done_cb=callback)
        else:
            self.StateQuaternionStateClient.send_goal_and_wait(goal_state)

    #set angular velocity
    def torque(self,vel):
        x,y,z = vel
        goal = self.get_superimposer_goal([None,None,None,x,y,z])
        self.SuperimposerClient.send_goal(goal)

    #set thruster velocity in local space (i.e. z is always heave)
    def forceLocal(self,vel):
        x,y = vel
        goal = self.get_superimposer_goal([x,y,None,None,None,None])
        self.SuperimposerClient.send_goal(goal)
    
    #stop all thrusters
    def kill(self):
        self.preemptCurrentAction()

    #stay still in place
    def stop_in_place(self):
        self.preemptCurrentAction()
        goal = self.get_state_goal([self.x,self.y,self.z,self.orientation.w,self.orientation.x,self.orientation.y,self.orientation.z],do_not_displace)
        self.StateQuaternionStateClient.send_goal_and_wait(goal)
