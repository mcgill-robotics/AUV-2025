#!/usr/bin/env python3

import rospy
import smach

from effort_states import Roll, Pause


if __name__ == '__main__':
    rospy.init_node('pool_testing_roll')
    sm =smach.StateMachine(outcomes=['finished'])

    with sm:
        smach.StateMachine.add('positive', Roll(effort=15), 
            transitions={'done':'pause'})
        smach.StateMachine.add('pause', Pause(duration=2.0), 
            transitions={'done':'negative'})
        smach.StateMachine.add('negative', Roll(effort=-15), 
            transitions={'done':'off'})
        smach.StateMachine.add('off', Pause(), 
            transitions={'done':'finished'})

    res = sm.execute()
