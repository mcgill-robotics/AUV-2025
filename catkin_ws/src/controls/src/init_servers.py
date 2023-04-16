#!/usr/bin/env python3

import rospy

from servers.StateServers import *
from servers.SuperimposerServer import *

def statePreempt():
    s.cancel()

def locPreempt():
    loc.cancel()

def globPreempt():
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