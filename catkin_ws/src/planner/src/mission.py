#!/usr/bin/env python3

import rospy
import smach

from submerge import SubmergingState
from std_msgs.msg import Float64

if __name__ == '__main__':
    rospy.init_node('mission_planner')
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('submerging', SubmergingState(), 
                transitions={'submerging_success': 'success', 'submerging_failure':'failure'})
        print(sm.get_active_states())

    res = sm.execute()
    print("mission complete, result: ", res)
