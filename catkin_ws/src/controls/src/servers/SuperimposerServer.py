#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from auv_msgs.msg import SuperimposerAction, SuperimposerFeedback, SuperimposerGoal, SuperimposerResult, StateGoal, StateAction
from std_msgs.msg import Float64, Bool
import time



class SuperimposerServer():

    def __init__(self) -> None:
        self.server = None

        self.goal = None

        self.position = None
        self.theta_x = None
        self.theta_y = None
        self.theta_z = None

        self.sub = rospy.Subscriber("pose",Pose,self.set_position)
        self.sub = rospy.Subscriber("state_theta_x",Float64,self.set_theta_x)
        self.sub = rospy.Subscriber("state_theta_y",Float64,self.set_theta_y)
        self.sub = rospy.Subscriber("state_theta_z",Float64,self.set_theta_z)
        
        self.pub_x_enable = rospy.Publisher('pid_x_enable', Bool, queue_size=50)
        self.pub_y_enable = rospy.Publisher('pid_y_enable', Bool, queue_size=50)
        self.pub_z_enable = rospy.Publisher('pid_z_enable', Bool, queue_size=50)
        self.pub_theta_x_enable = rospy.Publisher('pid_theta_x_enable', Bool, queue_size=50)
        self.pub_theta_y_enable = rospy.Publisher('pid_theta_y_enable', Bool, queue_size=50)
        self.pub_theta_z_enable = rospy.Publisher('pid_theta_z_enable', Bool, queue_size=50)
    
    def set_position(self,data):
        #print("updated pose")
        self.position = data.position
    
    def set_theta_x(self,data):
        self.theta_x = data.data
    def set_theta_y(self,data):
        self.theta_y = data.data
    def set_theta_z(self,data):
        self.theta_z = data.data



    def callback(self, goal):

        if(goal.displace):
            self.goal = self.displace_goal(goal)
        else:
            self.goal = goal
        #unset pids
        if(goal.do_surge):
            self.pub_x_enable.publish(Bool(False))
            self.pub_x.publish(self.goal.effort.force.x)
        if(goal.do_sway):
            #unset pids
            self.pub_y_enable.publish(Bool(False))
            self.pub_y.publish(self.goal.effort.force.y)
        if(goal.do_heave):
            #unset pids
            self.pub_z_enable.publish(Bool(False))
            self.pub_z.publish(self.goal.effort.force.z)
        if(goal.do_roll):
            #unset pids
            self.pub_theta_x_enable.publish(Bool(False))
            self.pub_theta_x.publish(self.goal.effort.torque.x)
        if(goal.do_pitch):
            #unset pids
            self.pub_theta_y_enable.publish(Bool(False))
            self.pub_theta_y.publish(self.goal.effort.torque.y)
        if(goal.do_yaw):
            #unset pids
            self.pub_theta_z_enable.publish(Bool(False))
            self.pub_theta_z.publish(self.goal.effort.torque.z)


        rate = rospy.rate(10)
        while True:
            rate.sleep()

        self.server.set_succeeded()
        # fb = SuperimposerFeedback()
        # fb.moving = True
        # self.server.publish_feedback(fb)

    def displace_goal(self,displace):
        if(self.goal == None):
            return displace

        new_goal = displace
        new_goal.force.x += self.goal.effort.force.x
        new_goal.force.y += self.goal.effort.force.y
        new_goal.force.z += self.goal.effort.force.z
        new_goal.torque.x += self.goal.effort.torque.x
        new_goal.torque.y += self.goal.effort.torque.y
        new_goal.torque.z += self.goal.effort.torque.z
        return new_goal
    
    def cancel(self):
        print("canceled")
        self.pub_x.publish(Float64(0))
        self.pub_y.publish(Float64(0))
        self.pub_z.publish(Float64(0))
        self.pub_theta_x.publish(Float64(0))
        self.pub_theta_y.publish(Float64(0))
        self.pub_theta_z.publish(Float64(0))
        self.server.set_succeeded()
        # print("hello")    
        # server = actionlib.SimpleActionClient('state_server', StateAction)
        # server.wait_for_server()
        # goal = StateGoal()
        # goal.position = self.position
        # goal.rotation.x = self.theta_x
        # goal.rotation.y = self.theta_y
        # goal.rotation.z = self.theta_z
        # goal.displace = False
        # goal.do_x, goal.do_y, goal.do_z, goal.do_theta_x, goal.do_theta_y, goal.do_theta_z = [True]*6
        # server.send_goal_and_wait(goal)
        

        
class LocalSuperimposerServer(SuperimposerServer):
    def __init__(self):
        super().__init__()
        self.server = actionlib.SimpleActionServer('superimposer_local_server', SuperimposerAction, execute_cb= self.callback, auto_start = False)
        #self.feedback = StateFeedback()
        #self.result = StateResult()

        self.pub_x = rospy.Publisher('surge', Float64, queue_size=50)
        self.pub_y = rospy.Publisher('sway', Float64, queue_size=50)
        self.pub_z = rospy.Publisher('heave', Float64, queue_size=50)
        self.pub_theta_x = rospy.Publisher('roll', Float64, queue_size=50)
        self.pub_theta_y = rospy.Publisher('pitch', Float64, queue_size=50)
        self.pub_theta_z = rospy.Publisher('yaw', Float64, queue_size=50)


        
        
        self.server.start()

class GlobalSuperimposerServer(SuperimposerServer):
    def __init__(self):
        super().__init__()
        self.server = actionlib.SimpleActionServer('superimposer_global_server', SuperimposerAction, execute_cb= self.callback, auto_start = False)
        #self.feedback = StateFeedback()
        #self.result = StateResult()

        self.pub_x = rospy.Publisher('global_x', Float64, queue_size=50)
        self.pub_y = rospy.Publisher('global_y', Float64, queue_size=50)
        self.pub_z = rospy.Publisher('global_z', Float64, queue_size=50)
        self.pub_theta_x = rospy.Publisher('roll', Float64, queue_size=50)
        self.pub_theta_y = rospy.Publisher('pitch', Float64, queue_size=50)
        self.pub_theta_z = rospy.Publisher('yaw', Float64, queue_size=50)

        self.server.start()
    