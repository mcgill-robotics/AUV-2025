#!/usr/bin/env python3

import rospy
from auv_msgs.msg import ThrusterCommand
from geometry_msgs.msg import Wrench

def wrench_to_thrust():
    '''
    wrench_to_thrust returns a function, configured with thrust
    characterisations, that maps a Wrench into thrust voltages
    '''
    stub = Wrench([0, 0, 0], [0, 0, 0]);
    return stub


if __name__ == '__main__':
    rospy.init_node('thrust_mapper')
    thrust_pub = rospy.Publisher('propulsion/thruster_cmd', ThrusterCommand, queue_size=5)
    sub = rospy.Subscriber('/effort', Wrench, wrench_to_thrust())
    rospy.spin()
