#!/usr/bin/env python3

import rospy

from servers.StateServers import *
from servers.SuperimposerServer import *

if __name__ == "__main__":
    rospy.init_node("state_server")
    s = StateServer()
    loc = LocalSuperimposerServer()
    glob = GlobalSuperimposerServer()
    rospy.spin()