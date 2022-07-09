#!/usr/bin/env python3

import actionlib
import rospy

from auv_msgs.msg import DirectAction, StateAction
from std_msgs.msg import Float64, Bool

from controllers import StateController, DirectController


class Router:

    def __init__(self):
        self.direct_server = actionlib.SimpleActionServer('direct', DirectAction, self.direct_goal_cb, False)
        self.state_server = actionlib.SimpleActionServer('state', StateAction, self.state_goal_cb, False)
        self.alloc_table = {'SURGE':None, 'SWAY':None, 'HEAVE':None}


    def direct_goal_cb(self, cmd):
        goal_dof = cmd.dof
        goal_type = cmd.ctl_type
        self.update_routes(goal_dof, goal_type)
        goal_controller = alloc_table[goal_dof]

        client = actionlib.SimpleActionClient('controller/' + goal_dof, DirectAction)
        client.send_goal(cmd, done_cb=self.direct_done_cb)


    def direct_done_cb(self, status, result):
        self.direct_server.set_succeeded()
        return


    def state_goal_cb(self, cmd):
        goal_dof = cmd.dof
        goal_type = cmd.ctl_type
        self.update_routes(goal_dof, goal_type)
        goal_controller = alloc_table[goal_dof]

        client = actionlib.SimpleActionClient('controller/' + goal_dof, StateAction)
        client.send_goal(cmd, done_cb=self.state_done_cb)

    
    def state_done_cb(self, status, result):
        self.state_server.set_succeeded()
        return


    def update_routes(self, goal_dof, goal_type):
        existing_controller = self.alloc_table[goal_dof]

        # deg. of freedom not as of yet controlled - create new controller
        if existing_controller is None:
            goal_controller = self.create_ctl(cmd.ctl_type)
            self.alloc_table[goal_dof] = goal_controller

        # existing controller of different type - pre-empt/overwite existing controller
        elif existing_controller.type != goal_type:
            existing_controller.finish()
            goal_controller = self.create_ctl(cmd.ctl_type)
            alloc_table[existing_controller.dof] = goal_controller

        # no changes necessary
        return


    def create_ctl(ctl_type):
        if ctl_type == 'DIRECT':
            ctl = DirectController()
        elif ctl_type == 'STATE':
            ctl = StateController()

        # start action server on controller
        ctl.start()
        return ctl


    def start(self):
        self.direct_server.start()
        self.state_server.start()


if __name__ == '__main__':
    rospy.init_node('router')
    router = Router()
    router.start()
    rospy.spin()

