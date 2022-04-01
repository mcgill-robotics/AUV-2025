#!/usr/bin/env python3

import rospy
import smach

from states import *

if __name__ == '__main__':
    rospy.init_node('stub')
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('submerge', DepthState(4.0), 
                transitions={'success': 'ascend', 'failure':'failure'})
        smach.StateMachine.add('ascend', DepthState(2.0), 
                transitions={'success': 'descend', 'failure':'failure'})
        smach.StateMachine.add('descend', DepthState(6.0), 
                transitions={'success': 'ascend', 'failure':'failure'})
        
    res = sm.execute()
    print("mission complete, result: ", res)
