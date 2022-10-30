#!/usr/bin/env python3

import rospy
import smach

from dof_states import Z

if __name__ == '__main__':
    rospy.init_node('pool_testing_pid_heave')
    sm =smach.StateMachine(outcomes=['finished'])

    with sm:
        smach.StateMachine.add('1m depth', Z(setpoint=-1.0), 
            transitions={'done':'2m depth'})
        smach.StateMachine.add('2m depth', Z(setpoint=-2.0), 
            transitions={'done':'1m depth'})
    res = sm.execute()
