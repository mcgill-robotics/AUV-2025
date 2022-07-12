#!/usr/bin/env python3

import rospy
import smach

from pooltest import DeadReckonMotion, Pause
from std_msgs.msg import Float64

class Surge(DeadReckonMotion):
    def __init__(self, effort, duration=10.0):
        super().__init__('surge', effort, duration)

class Heave(DeadReckonMotion):
    def __init__(self, effort, duration=10.0):
        super().__init__('heave', effort, duration)


if __name__ == '__main__':
    rospy.init_node('pool_testing_surge')
    sm =smach.StateMachine(outcomes=['finished'])

    with sm:
        smach.StateMachine.add('submerge', Heave(effort=-0.20, duration = 3.0),
            transitions={'done':'pause'})
        smach.StateMachine.add('pause', Pause(duration=2.0), 
            transitions={'done':'positive'})
        smach.StateMachine.add('positive', Surge(effort=0.20), 
            transitions={'done':'negative'})
        smach.StateMachine.add('negative', Surge(effort=-0.20), 
            transitions={'done':'off'})
        smach.StateMachine.add('off', Pause(), 
            transitions={'done':'finished'})

    res = sm.execute()
