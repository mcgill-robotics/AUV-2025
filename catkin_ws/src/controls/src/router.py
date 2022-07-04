#!/usr/bin/env python3

import actionlib
import rospy

from auv_msgs.msg import DirectAction, StateAction
from std_msgs.msg import Float64, Bool

from ctls import StateController, DirectController


class Router:

    def __init__(self):
        self.direct_server = actionlib.SimpleActionServer('direct_ctl', DirectControllerCommand, self.update_ctls, False)
        self.state_server = actionlib.SimpleActionServer('state_ctl', StateControllerCommand, self.update_ctls, False)

        self.alloc_table = {'SURGE':None, 'SWAY':None, 'HEAVE':None}


    def update_ctls(self, cmd):
        ctl_type = cmd.ctl_type
        curr_ctl = alloc_table[ctl_type]

        # logic to deal with the existing controller for the directioniable
        if curr_ctl.type == ctl_type:
            curr_ctl.update_command(cmd)

        else:
            alloc_table[curr_ctl.direction] = None
            status, result = curr_ctl.destroy()
            if status:
                self.new_ctl.set_succeeded(result)
            else:
                self.new_ctl.set_failed(result)

        # set current controller from cmd 
        if alloc_table[cmd.direction] is None:
            new_ctl = self.get_ctl(cmd)
            new_ctl.start()


    def get_ctl(cmd):
        ctl_type = cmd.type
        if ctl_type == 'DIRECT':
            return DirectController(cmd)
        elif ctl_type == 'STATE':
            return StateController(cmd)


    def up(self):
        # start each action server
        self.direct_server.start()
        self.state_server.start()

        rospy.Timer(rospy.Duration(0.1), self.publish)


    def publish(self):
        unique_ctls = set(self.alloc_table.values())
        for ctl in unique_ctls:
            ctl.publish()


if __name__ == '__main__':
    rospy.init_node('router')
    router = Router()
    router.up()
    rospy.spin()

