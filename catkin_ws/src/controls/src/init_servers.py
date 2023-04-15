import rospy

from servers.StateServers import *

if __name__ == "__main__":
    rospy.init_node("state_server")
    sc = StateControlActionServer()
    d = DisplaceServer()
    sup = SuperimposerServer()
    rospy.spin()