#!/usr/bin/env python3

import rospy
import smach

from dof_states import Surge, Yaw, Pause, descend, ascend, Pitch, Roll

if __name__ == '__main__':
    rospy.init_node('pool_testing_heave')
    sm =smach.StateMachine(outcomes=['finished'])

    with sm:
        smach.StateMachine.add('startup', Pause(duration=2.0), 
            transitions={'done':'descend1'})
        smach.StateMachine.add('descend1', descend(),
            transitions={'done':'yaw1'})
        smach.StateMachine.add('yaw1', Yaw(effort=15.0, duration=3.0), 
            transitions={'done' : 'pause1'})
        smach.StateMachine.add('pause1', Pause(duration=10.0),
            transitions={'done' : 'pitch1'})
        smach.StateMachine.add('pitch1', Pitch(effort=15.0, duration=3.0),
            transitions={'done' : 'pause2'})
        smach.StateMachine.add('pause2', Pause(duration=10.0),
            transitions={'done' : 'roll1'})
        smach.StateMachine.add('roll1', Roll(effort=15.0, duration=3.0),
            transitions={'done' : 'pause3'})
        smach.StateMachine.add('pause3', Pause(duration=10.0),
            transitions={'done' : 'ascend1'})
        smach.StateMachine.add('ascend1', ascend(duration=10.0),
            transitions={'done' : 'off'})
        smach.StateMachine.add('off', Pause(), 
            transitions={'done':'finished'})
        
    res = sm.execute()
