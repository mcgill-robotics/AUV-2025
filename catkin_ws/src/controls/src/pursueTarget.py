#!/usr/bin/env python3

import actionlib
import rospy

from std_msgs.msg import Float64
from auv_msgs.msg import PursueTargetAction

class pursue_target_server:
    epsilon = 0.05 # permissible error [m]

    def __init__(self):
        self.pub = rospy.Publisher('pid_setpoint', Float64, queue_size=50)
        self.sub = rospy.Subscriber('state', Float64, self.state_cb, queue_size=50)
        self.server = actionlib.SimpleActionServer('pursueTarget', PursueTargetAction, self.execute_cb, False)
        self.server.start()

    def execute_cb(self, goal):
        # compute effort using PID
        self.pub.publish(goal.depth)

    def state_cb(self, state):
        # TODO - possible race condition
        self.server.publish_feedback(state.depth)
        if self.goal.depth - state.depth < epsilon:
            self.server.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('pursueTargetServer')
    server = pursue_target_server()
    rospy.spin()

