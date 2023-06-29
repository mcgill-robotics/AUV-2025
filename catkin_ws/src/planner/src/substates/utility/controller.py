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


# predefined bools so we don't have to write these out everytime we want to get a new goal
do_xyz = [Bool(True),Bool(True),Bool(True),Bool(False),Bool(False),Bool(False)]
do_xy = [Bool(True),Bool(True),Bool(False),Bool(False),Bool(False),Bool(False)]
do_z = [Bool(False),Bool(False),Bool(True),Bool(False),Bool(False),Bool(False)]
do_txtytz = [Bool(False),Bool(False),Bool(False),Bool(True),Bool(True),Bool(True)]
do_tz = [Bool(False),Bool(False),Bool(False),Bool(False),Bool(False),Bool(True)]
do_all = [Bool(True),Bool(True),Bool(True),Bool(True),Bool(True),Bool(True)]
do_displace = Bool(True)
do_not_displace = Bool(False)


"""
Helper class for the planner. Takes in simple commands, converts them to 
goals and sends them to the control servers.
"""
class Controller:
    def __init__(self, get_header_time):
        print("starting controller")
        self.get_header_time = get_header_time

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer)
        self.tf_header = Header(frame_id="world_rotation")

        self.clients = []

        self.StateClient = actionlib.SimpleActionClient('state_server', StateAction)
        self.clients.append(self.StateClient)
        self.StateClient.wait_for_server()

        self.SuperimposerClient = actionlib.SimpleActionClient('superimposer_server', SuperimposerAction)
        self.clients.append(self.SuperimposerClient)
        self.SuperimposerClient.wait_for_server()

        self.x = None
        self.y = None
        self.z = None
        self.theta_x = None
        self.theta_y = None
        self.theta_z = None

        self.sub = rospy.Subscriber("pose",Pose,self.pos_cb)
        self.sub_theta_x = rospy.Subscriber("state_theta_x",Float64, self.theta_x_cb)
        self.sub_theta_y = rospy.Subscriber("state_theta_y",Float64, self.theta_y_cb)
        self.sub_theta_z = rospy.Subscriber("state_theta_z",Float64, self.theta_z_cb)

    def pos_cb(self,data):
        self.x = data.position.x
        self.y = data.position.y
        self.z = data.position.z

    def theta_x_cb(self,data):
        self.theta_x = data.data
    def theta_y_cb(self,data):
        self.theta_y = data.data
    def theta_z_cb(self,data):
        self.theta_z = data.data
    
    def transformLocalToGlobal(self,lx,ly,lz):
        """
        Performs a coordinate transformation from the auv body frame
        to the world frame.
        """
        trans = self.tf_buffer.lookup_transform("world_rotation", "auv_rotation", self.get_header_time())
        offset_local = Vector3(lx, ly, lz)
        self.tf_header.stamp = self.get_header_time()
        offset_local_stmp = Vector3Stamped(header=self.tf_header, vector=offset_local)
        offset_global = tf2_geometry_msgs.do_transform_vector3(offset_local_stmp, trans)
        return float(offset_global.vector.x), float(offset_global.vector.y), float(offset_global.vector.z)

    #method to easily get goal object
    def create_superimposer_goal(self,dofs,keepers,is_global):
        x,y,z,roll,pitch,yaw = dofs
        goal = SuperimposerGoal()
        goal.effort.force.x = x
        goal.effort.force.y = y
        goal.effort.force.z = z
        goal.effort.torque.x = roll
        goal.effort.torque.y = pitch
        goal.effort.torque.z = yaw
        goal.is_global = Bool(is_global)
        goal.do_x, goal.do_y, goal.do_z, goal.do_roll, goal.do_pitch, goal.do_yaw = keepers
        return goal
    
    #method to easily get goal object
    def create_state_goal(self,state,keepers,displace):
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

#     =============================
#     |       API FUNCTIONS       |
#     =============================

    #preempt the current action
    def preemptCurrentAction(self):
        for client in self.clients:
            if client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE]: client.cancel_goal()

    #rotate to this rotation
    def rotate(self,ang,callback=None):
        tx,ty,tz = ang
        if tx is None: tx = self.theta_x
        if ty is None: ty = self.theta_y
        if tz is None: tz = self.theta_z
        goal = self.create_state_goal([0,0,0,tx,ty,tz],do_txtytz,do_not_displace)
        if callback == None:
            self.StateClient.send_goal_and_wait(goal)
        else:
            self.StateClient.send_goal(goal,done_cb=callback)

    #move to setpoint
    def move(self,pos,callback=None):
        x,y,z = pos
        if x is None: x = self.x
        if y is None: y = self.y
        if z is None: z = self.z
        goal = self.create_state_goal([x,y,z,0,0,0],do_xyz,do_not_displace)
        if callback == None:
            self.StateClient.send_goal_and_wait(goal)
        else:
            self.StateClient.send_goal(goal,done_cb=callback)

    #move by this amount in world space
    def moveDelta(self,delta,callback=None):
        x,y,z = delta
        goal = self.create_state_goal([x,y,z,0,0,0],do_xyz,do_displace)
        if callback == None:
            self.StateClient.send_goal_and_wait(goal)
        else:
            self.StateClient.send_goal(goal,done_cb=callback)

    #rotate by this amount
    def rotateDelta(self,delta,callback=None):
        tx,ty,tz = delta
        goal = self.create_state_goal([0,0,0,tx,ty,tz],do_txtytz,do_displace)
        if callback == None:
            self.StateClient.send_goal_and_wait(goal)
        else:
            self.StateClient.send_goal(goal,done_cb=callback)

    #move by this amount in local space (i.e. z is always heave)
    def moveDeltaLocal(self,delta,callback=None):
        x,y,z = delta
        delta_gx, delta_gy, delta_gz  = self.transformLocalToGlobal(x, y, z)
        goal = self.create_state_goal([delta_gx, delta_gy, delta_gz, 0, 0, 0], do_xyz, do_displace)
        if callback == None:
            self.StateClient.send_goal_and_wait(goal)
        else:
            self.StateClient.send_goal(goal,done_cb=callback)

    #set angular velocity
    def torque(self,vel):
        x,y,z = vel
        goal = self.create_superimposer_goal([0,0,0,x,y,z],do_txtytz,is_global=True)
        self.SuperimposerClient.send_goal(goal)

    #set thruster velocity in global space (i.e. z is always up)
    def force(self,vel):
        x,y = vel
        goal = self.create_superimposer_goal([x,y,0,0,0,0],do_xy,is_global=True)
        self.SuperimposerClient.send_goal(goal)

    # !! IMPORTANT NOTE !! -> assumes AUV is relatively flat in orientation so as not to fight PIDs
    #set thruster velocity in local space (i.e. z is always heave)
    def forceLocal(self,vel):
        x,y = vel
        goal = self.create_superimposer_goal([x,y,0,0,0,0],do_xy,is_global=False)
        self.SuperimposerClient.send_goal(goal)
    
    #stop all thrusters
    def kill(self):
        goal = self.create_superimposer_goal([0,0,0,0,0,0],do_all,is_global=True)
        goal = self.create_superimposer_goal([0,0,0,0,0,0],do_all,is_global=False)
        self.SuperimposerClient.send_goal(goal)

    #stay still in place
    def stop_in_place(self):
        self.preemptCurrentAction()
        goal = self.create_state_goal([self.x,self.y,self.z,self.theta_x,self.theta_y,self.theta_z],do_all,do_not_displace)
        self.StateClient.send_goal_and_wait(goal)
