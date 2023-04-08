#!/usr/bin/env python3

import rospy
import smach

from sub_states import *

if __name__ == '__main__':
    rospy.init_node('mission_planner')
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('submerge', DepthState(4.0), 
                transitions={'success': 'success', 'failure':'failure'})

    res = sm.execute()
