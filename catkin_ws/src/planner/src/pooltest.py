#!/usr/bin/env python3

import rospy
import smach

from std_msgs.msg import Float64

class DeadReckonMotion(smach.State):
    def __init__(self, topic, effort, duration=2.0):
        super().__init__(outcomes=['done'])
        self.effort = Float64(effort)
        self.duration = duration
        self.pub = rospy.Publisher(topic, Float64, queue=50)
    
    def execute(self, ud):
        rospy.Timer(0.1, self.update)
        rospy.sleep(self.duration)
        return 'done'

    def update():
        self.pub.publish(self.effort) 

        
class Pause(smach.State):
    def __init__(self, duration=0.0):
        super().__init__(outcomes=['done'])
        self.duration = duration

    def execute(self, ud):
        rospy.sleep(self.duration)
        return 'done'


class Heave(DeadReckonMotion):
    def __init__(self, effort, duration=2.0):
        super().__init__('heave', effort, duration)


if __name__ == '__main__':
    rospy.init_node('pool_testing')
    sm = smach.StateMachine(outcomes=['finished']) 
    with sm:
        smach.StateMachine.add('submerge', Heave(effort=-0.20, duration=3.0), 
                transitions={'done':'pause'})
        smach.StateMachine.add('pause', Pause(duration=2.0), 
                transitions={'done':'ascend'})
        smach.StateMachine.add('ascend', Heave(effort=0.20), 
                transitions={'done':'descend'})
        smach.StateMachine.add('descend', Heave(effort=-0.20), 
                transitions={'done':'off'})
        smach.StateMachine.add('off', Pause(), 
                transitions={'done':'finished'})

    res = sm.execute()
