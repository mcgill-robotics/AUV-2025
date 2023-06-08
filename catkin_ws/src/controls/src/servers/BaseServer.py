#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64, Bool
import time

"""
This class servers as an abstract class for the action lib servers the controls use to
execute goals. In the process of developing the controls servers, I found that the I was
duplicating a lot of code so I made this base class to avoid issues of fixing a bug in one
server but not fixing it in another.

The methods this class have are mainly boiler plate. There are helper methods to establish
publishers and subscribers, a default preempt callback that sets the pids to the current position,
methods to check if a goal pose has been entered, a method to automatically turn off pids,
and a method to publish setpoints to the pids.
"""
class BaseServer():

    def __init__(self) -> None:
        print("starting server")
        self.cancelled = False
        self.goal = None
        


    def establish_pid_publishers(self):
        self.pub_z_pid = rospy.Publisher('z_setpoint', Float64, queue_size=50)
        self.pub_y_pid = rospy.Publisher('y_setpoint', Float64, queue_size=50)
        self.pub_x_pid = rospy.Publisher('x_setpoint', Float64, queue_size=50)
        self.pub_theta_x_pid = rospy.Publisher('theta_x_setpoint', Float64, queue_size=50)
        self.pub_theta_y_pid = rospy.Publisher('theta_y_setpoint', Float64, queue_size=50)
        self.pub_theta_z_pid = rospy.Publisher('theta_z_setpoint', Float64, queue_size=50)
        

    def establish_pid_enable_publishers(self):
        self.pub_x_enable = rospy.Publisher('pid_x_enable', Bool, queue_size=50)
        self.pub_y_enable = rospy.Publisher('pid_y_enable', Bool, queue_size=50)
        self.pub_z_enable = rospy.Publisher('pid_z_enable', Bool, queue_size=50)
        self.pub_theta_x_enable = rospy.Publisher('pid_theta_x_enable', Bool, queue_size=50)
        self.pub_theta_y_enable = rospy.Publisher('pid_theta_y_enable', Bool, queue_size=50)
        self.pub_theta_z_enable = rospy.Publisher('pid_theta_z_enable', Bool, queue_size=50)

    def establish_state_subscribers(self):
        self.position = Point(0,0,0)
        self.theta_x = 0
        self.theta_y = 0
        self.theta_z = 0
        self.sub = rospy.Subscriber("pose",Pose,self.set_position)
        self.sub = rospy.Subscriber("state_theta_x",Float64,self.set_theta_x)
        self.sub = rospy.Subscriber("state_theta_y",Float64,self.set_theta_y)
        self.sub = rospy.Subscriber("state_theta_z",Float64,self.set_theta_z)

    
    #callback for subscriber
    def set_position(self,data):
        #print("updated pose")
        self.position = data.position
    
    #callback for subscriber
    def set_theta_x(self,data):
        self.theta_x = data.data
    #callback for subscriber
    def set_theta_y(self,data):
        self.theta_y = data.data
    #callback for subscriber
    def set_theta_z(self,data):
        self.theta_z = data.data

    
    #generic cancel that publishes current position to pids to stay in place
    def cancel(self):
        self.cancelled = True
        self.pub_z_pid.publish(self.position.z)
        self.pub_y_pid.publish(self.position.y)
        self.pub_x_pid.publish(self.position.x)
        self.pub_theta_x_pid.publish(self.theta_x)
        self.pub_theta_y_pid.publish(self.theta_y)
        self.pub_theta_z_pid.publish(self.theta_z)
    
        # result = StateResult()
        # result.status = False
        self.server.set_succeeded()
    