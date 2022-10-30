#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

P = 0.1 #proportional factor for determining required surge

if __name__ == '__main__':
    current_x = None
    rospy.init_node('pid')
    publisher = rospy.Publisher('/surge', Float64, queue_size=5)

    def update_state(x):
        global current_x
        current_x = x

    def update_setpoint(target_x):
        global current_x
        if current_x == None: return
        out = Float64(P*(target_x-current_x))
        publisher.publish(out)

    state_subscriber = rospy.Subscriber('/state_x', Float64, update_state)
    setpoint_subscriber = rospy.Subscriber('/setpoint_x', Float64, update_setpoint)
    rospy.spin()
