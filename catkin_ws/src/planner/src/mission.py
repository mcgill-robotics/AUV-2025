#!/usr/bin/env python3

import rospy
import smach

from states import *
from std_msgs.msg import Float64

if __name__ == '__main__':
    rospy.init_node('mission_planner')
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('submerge', DepthState(4.0), 
                transitions={'success': 'success', 'failure':'failure'})
        print(sm.get_active_states())

    res = sm.execute()
