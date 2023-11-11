#!/usr/bin/env python3
import rospy
import smach
from substates.utility.controller import *
from substates.utility.state import *
    
rospy.init_node('eduard_demo_node')
state = StateTracker()
control = Controller(rospy.Time(0))

DEPTH = -2
DISTANCE1 = 5
DISTANCE2 = 2
DISTANCE3 = 5

class Submerge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fail', 'success'])

    def execute(self, userdata):
        control.move([0, 0, DEPTH], face_destination=True)
        control.rotateEuler([0, 0, 0])
        return 'success'
    
class CircleAround:
    class Forward(smach.State):
        def __init__(self, dist):
            smach.State.__init__(self, outcomes=['fail', 'success'])
            self.dist = dist

        def execute(self, userdata):
            control.moveDeltaLocal([self.dist, 0, 0])
            return 'success'
    
    class TurnLeft(smach.State):
        def __init__(self, angle):
            smach.State.__init__(self, outcomes=['fail', 'success'])
            self.angle = angle

        def execute(self, userdata):
            control.rotateDeltaEuler([0, 0, self.angle])
            return 'success'
    
class FinishedMission(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fail', 'success'])

    def execute(self, userdata):
        control.kill()
        return 'success'

mission_sm = smach.StateMachine(outcomes=['fail', 'success'])
circle_around_sm = smach.StateMachine(outcomes=['fail', 'success'])
with circle_around_sm:
    smach.StateMachine.add('FORWARD1', CircleAround.Forward(DISTANCE1), transitions={'fail':'fail', 'success':'ROTATE_LEFT1'})
    smach.StateMachine.add('ROTATE_LEFT1', CircleAround.TurnLeft(90), transitions={'fail':'fail', 'success':'FORWARD2'})
    smach.StateMachine.add('FORWARD2', CircleAround.Forward(DISTANCE2), transitions={'fail':'fail', 'success':'ROTATE_LEFT2'})
    smach.StateMachine.add('ROTATE_LEFT2', CircleAround.TurnLeft(90), transitions={'fail':'fail', 'success':'FORWARD3'})
    smach.StateMachine.add('FORWARD3', CircleAround.Forward(DISTANCE3), transitions={'fail':'fail', 'success':'success'})
with mission_sm:
    smach.StateMachine.add('SUBMERGE', Submerge(), transitions={'fail':'fail', 'success':'CIRCLE_AROUND'})
    smach.StateMachine.add('CIRCLE_AROUND', circle_around_sm, transitions={'fail':'fail', 'success':'FINISHED'})
    smach.StateMachine.add('FINISHED', FinishedMission(), transitions={'fail':'fail', 'success':'success'})

outcome = mission_sm.execute()
print(outcome)

