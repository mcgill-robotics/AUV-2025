#!/usr/bin/env python3

import rospy
import smach

from pooltest import DeadReckonMotion, Pause
from std_msgs.msg import Float64

class Roll(DeadReckonMotion):
    def __init__(self, effort, duration=10.0):
        super().__init__('roll', effort, duration)

class Heave(DeadReckonMotion):
    def __init__(self, effort, duration=10.0):
        super().__init__('heave', effort, duration)


if __name__ == '__main__':
    rospy.init_node('pool_testing_roll')
    sm =smach.StateMachine(outcomes=['finished'])

    with sm:
        smach.StateMachine.add('submerge', Heave(effort=-0.20, duration = 3.0),
            transitions={'done':'pause'})
        smach.StateMachine.add('pause', Pause(duration=2.0), 
            transitions={'done':'positive'})
        smach.StateMachine.add('positive', Roll(effort=0.20), 
            transitions={'done':'negative'})
        smach.StateMachine.add('negative', Roll(effort=-0.20), 
            transitions={'done':'off'})
        smach.StateMachine.add('off', Pause(), 
            transitions={'done':'finished'})

    res = sm.execute()
