#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64, Bool
import time


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
    
    def enable_pids(self,goal):
        if(goal.do_x.data):
            self.pub_x_enable.publish(Bool(True))
        if(goal.do_y.data):
            self.pub_y_enable.publish(Bool(True))
        if(goal.do_z.data):
            self.pub_z_enable.publish(Bool(True))
        if(goal.do_theta_x.data):
            self.pub_theta_x_enable.publish(Bool(True))
        if(goal.do_theta_y.data):
            self.pub_theta_y_enable.publish(Bool(True))
        if(goal.do_theta_z.data):
            self.pub_theta_z_enable.publish(Bool(True))

        

    def wait_for_settled(self,position,rotation):
        interval = 1

        settled = False
        print("waiting for settled")
        while not settled and not self.cancelled:
            #print("hi")
            start = time.time()
            while not self.cancelled and self.check_status(position,rotation):
                if(time.time() - start > interval):
                    settled = True
                    break
                rospy.sleep(0.01)
        print("settled")

    #true if auv is in goal position
    def check_status(self,position,rotation):
        if(self.position == None or self.theta_x == None or self.theta_y == None or self.theta_z == None):
            return False

        tolerance_position = 0.5
        tolerance_orientation = 1


        x_diff = (not self.goal.do_x.data) or abs(self.position.x - position.x) <= tolerance_position
        y_diff = (not self.goal.do_y.data) or abs(self.position.y - position.y) <= tolerance_position
        z_diff = (not self.goal.do_z.data) or abs(self.position.z - position.z) <= tolerance_position
        theta_x_diff = (not self.goal.do_theta_x.data) or abs(self.theta_x - rotation.x) <= tolerance_orientation
        theta_y_diff = (not self.goal.do_theta_y.data) or abs(self.theta_y - rotation.y) <= tolerance_orientation
        theta_z_diff = (not self.goal.do_theta_z.data) or abs(self.theta_z - rotation.z) <= tolerance_orientation


        return x_diff and y_diff and z_diff and theta_x_diff and theta_y_diff and theta_z_diff


    def publish_setpoints(self, position, rotation):
        if (self.goal.do_x.data):
            self.pub_x_pid.publish(Float64(position.x))
        if (self.goal.do_y.data):
            self.pub_y_pid.publish(Float64(position.y))
        if(self.goal.do_z.data):
            self.pub_z_pid.publish(Float64(position.z))
        if(self.goal.do_theta_x.data):
            self.pub_theta_x_pid.publish(Float64(rotation.x))
        if(self.goal.do_theta_y.data):
            self.pub_theta_y_pid.publish(Float64(rotation.y))
        if(self.goal.do_theta_z.data):
            self.pub_theta_z_pid.publish(Float64(rotation.z))