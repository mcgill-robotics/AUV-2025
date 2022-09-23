#!/usr/bin/env python3

import rospy
import smach

from dof_states import Heave, Pause

if __name__ == '__main__':
    rospy.init_node('pool_testing_heave')
    sm =smach.StateMachine(outcomes=['finished'])

    with sm:
        smach.StateMachine.add('positive', Heave(effort=0.2), 
            transitions={'done':'pause'})
        smach.StateMachine.add('pause', Pause(duration=2.0), 
            transitions={'done':'negative'})
        smach.StateMachine.add('negative', Heave(effort=-0.2), 
            transitions={'done':'off'})
        smach.StateMachine.add('off', Pause(), 
            transitions={'done':'finished'})

    res = sm.execute()
