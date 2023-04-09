import rospy

from servers.MoveServers import *

if __name__ == "__main__":
    rospy.init_node("state_server")
    s = StateControlActionServer()
    d = DisplaceServer()
    rospy.spin()