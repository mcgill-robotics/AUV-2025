#!/usr/bin/env python3

import rospy
import smach

from pooltest import SurgeState 
from std_msgs.msg import Float64

if __name__ == '__main__':
    rospy.init_node('pool_test_1_surge')
    sm =smach.StateMachine(outcomes=['finished', 'failure'])

    with sm:
        # direct motion surge for 5s
        smach.StateMachine.add('surge_positive', 
                SurgeState(effort=0.20, duration = 5.0),
                transitions={'succeeded':'finished', 
                    'preempted':'failure', 
                    'aborted':'failure'})

    res = sm.execute()
