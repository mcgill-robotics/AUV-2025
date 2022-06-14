#!/usr/bin/env python3

import actionlib
import rospy

from auv_msgs.msg import DirectAction, StateAction
from std_msgs.msg import Float64, Bool

from ctls import StateController, DirectController


class Router:

    def __init__(self):
        self.direct_server = actionlib.SimpleActionServer('direct_ctl', DirectControllerCommandArray, self.direct_cb, False)
        self.state_server = actionlib.SimpleActionServer('state_ctl', StateControllerCommandArray, self.state_cb, False)

        active_ctls = []
        alloc_table = {'SURGE':None, 'SWAY':None, 'HEAVE':None}


    def direct_cb(self, cmd_array):
        self.update_ctl(cmd_array, 'DIRECT')


    def state_cb(self, cmd_array):
        self.update_ctl(cmd_array, 'STATE')

    def update_ctls(self, ctl_type):
        for cmd in cmd_array:
            curr_ctl = alloc_table[cmd.var]

            if curr_ctl is None:
                new_ctl = self.get_ctl(cmd)
                self.active_ctls.append(new_ctl)

            elif curr_ctl.type == ctl_type:
                curr_ctl.new_command(cmd)

            else:
                active_ctl.remove(curr_ctl)
                alloc_table[curr_ctl.var] = None
                    
                status, result = curr_ctl.release()
                if status:
                    self.new_ctl.set_succeeded(result)
                else:
                    self.new_ctl.set_failed(result)


    def get_ctl(ctl_type, cmd):
        if ctl_type == 'DIRECT':
            return DirectController(cmd)
        elif ctl_type == 'STATE':
            return StateController(cmd)


    def up(self):
        # start each action server
        self.direct_server.start()
        self.state_server.start()

        rospy.Timer(rospy.Duration(0.1), self.update)


    def update(self):
        for ctl in self.active_ctls:
            ctl.publish()


if __name__ == '__main__':
    rospy.init_node('router')
    router = Router()
    router.up()
    rospy.spin()

