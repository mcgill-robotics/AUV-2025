#!/usr/bin/env python3

import rospy

from .servers.superimposer_server import SuperimposerServer
from .servers.state_quaternion_pid_server import StateQuaternionServer

#define preempt callbacks using the cancel methods. This is necessary because action lib does not
#allow methods to be callback function for preempting.
def supPreempt():
    print("preempting loc")
    sup.cancel()

def quatPreempt():
    print("preempting quat")
    quat.cancel()

if __name__ == "__main__":
    rospy.init_node("server manager", log_level=rospy.DEBUG)
    sup = SuperimposerServer()
    sup.server.register_preempt_callback(supPreempt)
    quat = StateQuaternionServer()
    quat.server.register_preempt_callback(quatPreempt)
    rospy.spin()