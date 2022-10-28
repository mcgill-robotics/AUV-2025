#!/usr/bin/env python3

import rospy
import smach

from dof_states import Pitch, Pause


if __name__ == '__main__':
    rospy.init_node('pool_testing_pitch')
    sm =smach.StateMachine(outcomes=['finished'])

    with sm:
        smach.StateMachine.add('positive', Pitch(effort=20.0), 
            transitions={'done':'pause'})
        smach.StateMachine.add('pause', Pause(duration=2.0), 
            transitions={'done':'negative'})
        smach.StateMachine.add('negative', Pitch(effort=-20.0), 
            transitions={'done':'off'})
        smach.StateMachine.add('off', Pause(), 
            transitions={'done':'finished'})

    res = sm.execute()
