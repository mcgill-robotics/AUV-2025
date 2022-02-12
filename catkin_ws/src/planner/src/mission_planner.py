#! /usr/bin/env python3

import rospy
import smach

from submerge import SubmergingState

if __name__ == '__main__':
    rospy.init_node('mission_planner')
    with smach.StateMachine(outcomes=['success', 'failure']) as sm:

        smach.StateMachine.add('submerging', SubmergingState(), 
                transitions={'success': 'success', 'failure':'failure'})

    res = sm.execute()
    print("mission complete, result: ", res)

