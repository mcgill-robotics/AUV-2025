#!/usr/bin/env python3

import rospy

# from servers.StateServers import *
# from servers.SuperimposerServer import *
from servers.state_server import StateServer
from servers.superimposer_server import SuperimposerServer
from servers.quaternion_pid_server_nasa import QuaternionServer

#define preempt callbacks using the cancel methods. This is necessary because action lib does not
#allow methods to be callback function for preempting.
def statePreempt():
    print("preempting state")
    s.cancel()

def supPreempt():
    print("preempting loc")
    sup.cancel()

def quatPreempt():
    print("preempting quat")
    quat.cancel()


if __name__ == "__main__":
    rospy.init_node("server manager", log_level=rospy.DEBUG)
    s = StateServer()
    s.server.register_preempt_callback(statePreempt)
    sup = SuperimposerServer()
    sup.server.register_preempt_callback(supPreempt)
    quat = QuaternionServer()
    quat.server.register_preempt_callback(quatPreempt)
    rospy.spin()