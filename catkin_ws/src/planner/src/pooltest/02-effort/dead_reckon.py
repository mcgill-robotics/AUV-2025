#!/usr/bin/env python3

import rospy
import smach

from effort_states import Surge, Yaw, Pause

if __name__ == '__main__':
    rospy.init_node('pool_testing_heave')
    sm =smach.StateMachine(outcomes=['finished'])

    with sm:
        smach.StateMachine.add('startup', Pause(duration=2.0), 
            transitions={'done':'surge1'})
        smach.StateMachine.add('surge1', Surge(effort=50, duration=5.0), 
            transitions={'done':'pause1'})
        smach.StateMachine.add('pause1', Pause(duration=2.0), 
            transitions={'done':'yaw'})
        smach.StateMachine.add('yaw', Yaw(effort=50, duration=5.0), 
            transitions={'done':'pause2'})
        smach.StateMachine.add('pause2', Pause(duration=2.0), 
            transitions={'done':'surge2'})
        smach.StateMachine.add('surge2', Surge(effort=50, duration=5.0), 
            transitions={'done':'off'})
        smach.StateMachine.add('off', Pause(), 
            transitions={'done':'finished'})

    res = sm.execute()
