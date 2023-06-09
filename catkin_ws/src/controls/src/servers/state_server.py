#!/usr/bin/env python3


import rospy

import actionlib
from geometry_msgs.msg import Point
from auv_msgs.msg import StateAction, StateFeedback, StateResult
from std_msgs.msg import Float64, Bool
import time
from servers.base_server import BaseServer

"""
This server class executes a state goal, which has a goal pose to enter.
The goal pose can be considered a global position, or a displacement within
the world reference frame.
"""
class StateServer(BaseServer):
    def __init__(self):
        super().__init__()
        self.establish_pid_publishers()
        self.establish_pid_enable_publishers()
        self.establish_state_subscribers()
        self.server = actionlib.SimpleActionServer('state_server', StateAction, execute_cb= self.callback, auto_start = False)
        self.server.start()

    def callback(self, goal):
        """
        Execute callback for this server. Executes the goal.
        If the goal is a displace, calculates a global pose to move to.
        Sets the pids then waits for the auv to enter the pose before
        setting the state to succeeded.
        """
        print("got a message")
        # set the PIDs
        #print(goal.pose)
        self.goal = goal
        self.enable_pids(goal)
        if(goal.displace.data):
            self.goal = self.dispalce_goal(goal)

        print(goal.position,goal.rotation)
        self.publish_setpoints()

        # monitor when reached pose
        self.wait_for_settled()

        # rospy.loginfo("Succeeded")
        if(not self.cancelled):
            self.server.set_succeeded()
    
    def dispalce_goal(self, goal):
        """
        Takes in a displacement goal pose and returns the corresponding
        world frame pose.
        """
        goal.position.x += self.position.x
        goal.position.y += self.position.y
        goal.position.z += self.position.z

        goal.rotation.x += self.theta_x
        goal.rotation.y += self.theta_y
        goal.rotation.z += self.theta_z

 
        return goal
    
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

        

    def wait_for_settled(self):
        interval = 1

        settled = False
        print("waiting for settled")
        while not settled and not self.cancelled:
            #print("hi")
            start = time.time()
            while not self.cancelled and self.check_status():
                if(time.time() - start > interval):
                    settled = True
                    break
                rospy.sleep(0.01)
        print("settled")

    #true if auv is in goal position
    def check_status(self):
        if(self.position == None or self.theta_x == None or self.theta_y == None or self.theta_z == None):
            return False

        tolerance_position = 0.5
        tolerance_orientation = 1


        x_diff = (not self.goal.do_x.data) or abs(self.position.x - self.goal.position.x) <= tolerance_position
        y_diff = (not self.goal.do_y.data) or abs(self.position.y - self.goal.position.y) <= tolerance_position
        z_diff = (not self.goal.do_z.data) or abs(self.position.z - self.goal.position.z) <= tolerance_position
        theta_x_diff = (not self.goal.do_theta_x.data) or abs(self.theta_x - self.goal.rotation.x) <= tolerance_orientation
        theta_y_diff = (not self.goal.do_theta_y.data) or abs(self.theta_y - self.goal.rotation.y) <= tolerance_orientation
        theta_z_diff = (not self.goal.do_theta_z.data) or abs(self.theta_z - self.goal.rotation.z) <= tolerance_orientation

        return x_diff and y_diff and z_diff and theta_x_diff and theta_y_diff and theta_z_diff


    def publish_setpoints(self):
        if (self.goal.do_x.data):
            self.pub_x_pid.publish(Float64(self.goal.position.x))
        if (self.goal.do_y.data):
            self.pub_y_pid.publish(Float64(self.goal.position.y))
        if(self.goal.do_z.data):
            self.pub_z_pid.publish(Float64(self.goal.position.z))
        if(self.goal.do_theta_x.data):
            self.pub_theta_x_pid.publish(Float64(self.goal.rotation.x))
        if(self.goal.do_theta_y.data):
            self.pub_theta_y_pid.publish(Float64(self.goal.rotation.y))
        if(self.goal.do_theta_z.data):
            self.pub_theta_z_pid.publish(Float64(self.goal.rotation.z))

