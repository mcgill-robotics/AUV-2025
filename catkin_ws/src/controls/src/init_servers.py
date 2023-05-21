#!/usr/bin/env python3

import rospy

# from servers.StateServers import *
# from servers.SuperimposerServer import *
from servers.BaseServer import *

#define preempt callbacks using the cancel methods. This is necessary because action lib does not
#allow methods to be callback function for preempting.
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
    rospy.init_node("server manager", log_level=rospy.DEBUG)
    s = StateServer()
    s.server.register_preempt_callback(statePreempt)
    loc = LocalSuperimposerServer()
    loc.server.register_preempt_callback(locPreempt)
    glob = GlobalSuperimposerServer()
    glob.server.register_preempt_callback(globPreempt)
    rospy.spin()