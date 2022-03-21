#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import roslib
from pooltest import Pause
from std_msgs.msg import Float64


class Yaw(smach.State):

    def __init__(self, effort, duration):
        super().__init__(outcomes=['done'])
        self.effort = Float64(effort)
        self.duration = duration
        self.pub_yaw = rospy.Publisher('yaw', Float64, queue_size=50)

    def execute(self, userdata):
        rospy.Timer(rospy.Duration(0.1), self.update)
        rospy.sleep(self.duration)
        return 'done'

    def update(self, effort):
        self.pub_yaw.publish(self.effort)

def main():
        rospy.init_node('smach_yaw_tester')
        sm = smach.StateMachine(outcomes=['finished'])
        with sm:
            smach.StateMachine.add('ClockWise', Yaw(effort=-.5, duration=3.0), transitions={'done':'pause'})
            smach.StateMachine.add('pause', Pause(duration=1.5), transitions={'done':'CounterClockWise'})
            smach.StateMachine.add('CounterClockWise', Yaw(effort=.8, duration=3.0), transitions={'done':'off'})
            smach.StateMachine.add('off', Pause(), transitions={'done':'finished'})
        outcome = sm.execute()
    
if __name__ == '__main__':
    main()
