#!/usr/bin/env python3

import rospy

from servers.StateServers import *
from servers.SuperimposerServer import *

def statePreempt():
    print("preempting state")
    s.cancel()

def locPreempt():
    print("preempting loc")
    loc.cancel()

def globPreempt():
    print("preempting glob")
    glob.cancel()

if __name__ == "__main__":
    rospy.init_node("state_server")
    s = StateServer()
    s.server.register_preempt_callback(statePreempt)
    loc = LocalSuperimposerServer()
    loc.server.register_preempt_callback(locPreempt)
    glob = GlobalSuperimposerServer()
    glob.server.register_preempt_callback(globPreempt)
    rospy.spin()