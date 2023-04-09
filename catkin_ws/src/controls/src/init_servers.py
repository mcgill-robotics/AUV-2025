import rospy

from state_controller_action_server import StateControlActionServer
from displace_server import DisplaceServer

if __name__ == "__main__":
    rospy.init_node("state_server")
    s = StateControlActionServer()
    d = DisplaceServer()
    rospy.spin()