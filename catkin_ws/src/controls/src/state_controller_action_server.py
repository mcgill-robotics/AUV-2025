#! /usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose
from auv_msgs.msg import StateAction, StateFeedback, StateResult
from std_msgs.msg import Float64

class StateControlActionServer():

    def __init__(self) -> None:
        self.server = actionlib.SimpleActionServer('state_control_action', StateAction, execute_cb= self.callback, auto_start = False)
        self.feedback = StateFeedback()
        self.result = StateResult()
        self.pub = rospy.Publisher('z_setpoint', Float64, queue_size=50)
        self.depth = 0
        self.depth_sub = rospy.Subscriber('state_z', Float64, self.move_position)
        self.server.start()


    # receives a new goal 
    def callback(self, goal):

        # set the PIDs
        self.pub.publish(goal.setpoint)

        # monitor when reached pose
        while(self.depth < goal.setpoint - 0.5 or self.depth > goal.setpoint + 0.5):
            continue

        self.result = self.feedback
        rospy.loginfo("Succeeded")
        self.server.set_succeeded(self.result)

    
    def move_position(self, value):
        
        self.depth = value.data


if __name__ == "__main__":
    rospy.init_node("state_control_action_server")
    s = StateControlActionServer()
    rospy.spin()


