#!/usr/bin/env python3

import actionlib
import rospy

from auv_msgs.msg import PursueTargetAction, PursueTargetFeedback, PursueTargetResult
from std_msgs.msg import Float64, Bool

class Pursue_Target_Server:
    epsilon = 0.05 # permissible error [m]
    state_z = None # depth [m] - initially unspecified

    def state_cb(state):
        Pursue_Target_Server.state_z = state

    def __init__(self):
        self.pub_setpoint = rospy.Publisher('setpoint_z', Float64, queue_size=50)
        self.server = actionlib.SimpleActionServer('pursueTarget', PursueTargetAction, self.execute_cb, False)
        rospy.sleep(1.0) # hack to avoid bringup issues 
        self.server.start()

    def execute_cb(self, goal):
        # compute effort using PID
        self.goal = goal
        self.pub_setpoint.publish(goal.target_depth)

        while True:
            if Pursue_Target_Server.state_z is None:
                continue

            feedback = PursueTargetFeedback(Pursue_Target_Server.state_z)
            self.server.publish_feedback(feedback)

            target_state = self.goal.target_depth.data
            curr_state = Pursue_Target_Server.state_z.data
            err_margin = Pursue_Target_Server.epsilon
            if abs(target_state - curr_state) < err_margin:
                break

        self.server.set_succeeded(PursueTargetResult(Pursue_Target_Server.state_z))
        self.goal = None

if __name__ == '__main__':
    rospy.init_node('pursueTargetServer')
    rospy.Subscriber('state_z', Float64, Pursue_Target_Server.state_cb, queue_size=50)
    server = Pursue_Target_Server()
    pub_pid_enable = rospy.Publisher('enable', Bool, queue_size=50)
    pub_pid_enable.publish(Bool(True)) # enable PID
    rospy.spin()

