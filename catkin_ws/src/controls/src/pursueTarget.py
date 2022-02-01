#!/usr/bin/env python3

import actionlib
import rospy

from auv_msgs.msg import PursueTargetAction, PursueTargetFeedback, PursueTargetResult
from std_msgs.msg import Float64

class Pursue_Target_Server:
    epsilon = 0.05 # permissible error [m]
    state = None # depth [m] - initially unspecified

    def state_cb(state):
        Pursue_Target_Server.state = state

    def __init__(self):
        self.pub = rospy.Publisher('setpoint', Float64, queue_size=50)
        self.server = actionlib.SimpleActionServer('pursueTarget', PursueTargetAction, self.execute_cb, False)
        self.server.start()

    def execute_cb(self, goal):
        # compute effort using PID
        self.goal = goal
        self.pub.publish(goal.target_depth)

        while True:
            if Pursue_Target_Server.state is None:
                continue

            feedback = PursueTargetFeedback(self.state)
            self.server.publish_feedback(feedback)

            target_state = self.goal.target_depth.data
            curr_state = Pursue_Target_Server.state.data
            err_margin = Pursue_Target_Server.epsilon
            if abs(target_state - curr_state) < err_margin:
                break

        self.server.set_succeeded(PursueTargetResult(Pursue_Target_Server.state))
        self.goal = None

if __name__ == '__main__':
    rospy.init_node('pursueTargetServer')
    rospy.Subscriber('state', Float64, Pursue_Target_Server.state_cb, queue_size=50)
    server = Pursue_Target_Server()
    rospy.spin()

