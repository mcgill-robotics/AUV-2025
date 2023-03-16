#!/usr/bin/env python3

import rospy
import smach

from dof_states import Surge, Rotate, Pause, descend, ascend

if __name__ == '__main__':
    rospy.init_node('pool_testing_heave')
    sm =smach.StateMachine(outcomes=['finished'])

    with sm:
        smach.StateMachine.add('startup', Pause(duration=2.0), 
            transitions={'done':'descend'})
        smach.StateMachine.add('descend', descend(), 
            transitions={'done':'surge1'})
        smach.StateMachine.add('surge1', Surge(effort=15, duration=5.0), 
            transitions={'done':'pause1'})
        smach.StateMachine.add('pause1', Pause(duration=2.0), 
            transitions={'done':'rotate'})
        smach.StateMachine.add('rotate', Rotate(effort=15), 
            transitions={'done':'pause2'})
        smach.StateMachine.add('pause2', Pause(duration=2.0), 
            transitions={'done':'surge2'})
        smach.StateMachine.add('surge2', Surge(effort=15, duration=5.0), 
            transitions={'done':'ascend'})
        smach.StateMachine.add('ascend', ascend(), 
            transitions={'done':'off'})
        smach.StateMachine.add('off', Pause(), 
            transitions={'done':'finished'})

    res = sm.execute()
