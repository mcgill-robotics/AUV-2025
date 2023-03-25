#!/usr/bin/env python3

import rospy
import smach

from dof_states import Surge, Yaw, Pause, descend, ascend, Pitch, Roll, Rotate

if __name__ == '__main__':
    rospy.init_node('pool_testing_heave')
    sm =smach.StateMachine(outcomes=['finished'])

    with sm:
        smach.StateMachine.add('startup', Pause(duration=2.0), 
            transitions={'done':'descend1'})
        smach.StateMachine.add('descend1', descend(),
            transitions={'done':'surge1'})
        smach.StateMachine.add('surge1', Surge(effort=15, duration=5.0), # surge 1
            transitions={'done' : 'pause1'})
        smach.StateMachine.add('pause1', Pause(duration=2.0), # pause 1
            transitions={'done' : 'rotate1'})
        smach.StateMachine.add('rotate1', Rotate(effort=15), # 90 degrees
            transitions={'done' : 'pause2'})
        smach.StateMachine.add('pause1', Pause(duration=2.0), # pause 2
            transitions={'done' : 'surge2'})
        smach.StateMachine.add('surge2', Surge(effort=15, duration=5.0), # surge 2
            transitions={'done' : 'pause3'})
        smach.StateMachine.add('pause3', Pause(duration=2.0), # pause 3
            transitions={'done' : 'rotate2'})
        smach.StateMachine.add('rotate2', Rotate(effort=15),  # 90 degrees
            transitions={'done' : 'pause4'})
        smach.StateMachine.add('pause4', Pause(duration=2.0), # pause 4
            transitions={'done' : 'surge2'})
        smach.StateMachine.add('surge2', Surge(effort=15, duration=5.0), # surge 3
            transitions={'done' : 'pause5'})
        smach.StateMachine.add('pause5', Pause(duration=2.0), # pause 5
            transitions={'done' : 'rotate3'})
        smach.StateMachine.add('rotate3', Rotate(effort=15),  # 90 degrees
            transitions={'done' : 'pause6'})
        smach.StateMachine.add('pause6', Pause(duration=2.0), # pause 6
            transitions={'done' : 'surge4'})
        smach.StateMachine.add('surge4', Surge(effort=15, duration=5.0), # surge 4
            transitions={'done' : 'pause7'})
        smach.StateMachine.add('pause6', Pause(duration=2.0), # pause 7
            transitions={'done' : 'ascend1'})  
        smach.StateMachine.add('ascend1', ascend(duration=10.0),
            transitions={'done' : 'off'})
        smach.StateMachine.add('off', Pause(), 
            transitions={'done':'finished'})
        
    res = sm.execute()
