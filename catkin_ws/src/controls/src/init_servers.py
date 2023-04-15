import rospy

from servers.StateServers import *
from servers.SuperimposerServer import *

if __name__ == "__main__":
    rospy.init_node("state_server")
    sc = StateControlActionServer()
    d = DisplaceServer()
    loc = LocalSuperimposerServer()
    glob = GlobalSuperimposerServer()
    rospy.spin()