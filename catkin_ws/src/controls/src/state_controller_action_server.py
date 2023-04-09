#! /usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from auv_msgs.msg import StateAction, StateFeedback, StateResult
from std_msgs.msg import Float64



import math
 


class StateControlActionServer():

    def __init__(self) -> None:
        self.server = actionlib.SimpleActionServer('state_server', StateAction, execute_cb= self.callback, auto_start = False)
        #self.feedback = StateFeedback()
        #self.result = StateResult()

        #self.pub_x = rospy.Publisher('', Float64, queue_size=50)
        #self.pub_y = rospy.Publisher('', Float64, queue_size=50)
        self.pub_z = rospy.Publisher('z_setpoint', Float64, queue_size=50)
        self.pub_theta_x = rospy.Publisher('theta_x_setpoint', Float64, queue_size=50)
        self.pub_theta_y = rospy.Publisher('theta_y_setpoint', Float64, queue_size=50)
        self.pub_theta_z = rospy.Publisher('theta_z_setpoint', Float64, queue_size=50)
        self.depth = 0
        
        self.server.start()

        self.position = None
        self.theta_x = None
        self.theta_y = None
        self.theta_z = None

        self.sub = rospy.Subscriber("pose",Pose,self.set_position)
        self.sub = rospy.Subscriber("state_theta_x",Float64,self.set_theta_x)
        self.sub = rospy.Subscriber("state_theta_y",Float64,self.set_theta_y)
        self.sub = rospy.Subscriber("state_theta_z",Float64,self.set_theta_z)
        
    
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
        #print("got a message")
        # set the PIDs
        #print(goal.pose)
        goal_position = goal.position
        goal_rotation = goal.rotation

        self.publish_setpoints(goal_position,goal_rotation)

        # monitor when reached pose
        while(not self.check_status(goal_position,goal_rotation)):
            rospy.sleep(0.1)

        result = StateResult()
        result.status.data = True
        rospy.loginfo("Succeeded")
        self.server.set_succeeded(result)

    def check_status(self,position,rotation):
        if(self.position == None or self.theta_x == None or self.theta_y == None or self.theta_z == None):
            return False

        tolerance_position = 0.5
        tolerance_orientation = 1

        #x_diff = abs(self.position.x - position.x) <= tolerance_position
        #y_diff = abs(self.position.y - position.y) <= tolerance_position
        z_diff = abs(self.position.z - position.z) <= tolerance_position
        theta_x_diff = abs(self.theta_x - rotation.x) <= tolerance_orientation
        theta_y_diff = abs(self.theta_y - rotation.y) <= tolerance_orientation
        theta_z_diff = abs(self.theta_z - rotation.z) <= tolerance_orientation

        return z_diff and theta_x_diff and theta_y_diff and theta_z_diff # and x_diff and y_diff


    def publish_setpoints(self,position,rotation):
        #self.pub_x.Publish(Float64(position.x))
        #self.pub_y.Publish(Float64(position.y))
        self.pub_z.publish(Float64(position.y))
        self.pub_theta_x.publish(Float64(rotation.x))
        self.pub_theta_y.publish(Float64(rotation.y))
        self.pub_theta_z.publish(Float64(rotation.z))
        #print("published setpoints")


if __name__ == "__main__":
    rospy.init_node("state_server")
    s = StateControlActionServer()
    rospy.spin()


