#!/usr/bin/env python3

import rospy
import smach

from substates.breadth_first_search import *
from substates.grid_search import *
from substates.linear_search import *
from substates.navigate_lane_marker import *
from substates.test_submerged_rotations import *
from substates.utility.controller import Controller
from substates.quali import *

def descend(depth):
    descended = False
    def done():
        global descended
        descended = True
    control.moveDelta((0, 0, depth), done)
    while not descended: rospy.sleep(0.1)

def endMission(msg="Shutting down mission planner."):
    print(msg)
    control.preemptCurrentAction()
    control.velocity((0,0,0))
    control.angularVelocity((0,0,0))

def testRotationsMission():
    descend(depth=-2.0)
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('test_submerged_rotations', TestSubmergedRotations(hold_time = 5.0, control=control), 
                transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished rotation test mission.")

def laneMarkerGridSearchMission():
    descend(depth=-0.5)
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('gridsearch', GridSearch(timeout=60, target_classes=[0], control=control), 
                transitions={'success': 'navigateLaneMarker', 'failure':'failure'})
        smach.StateMachine.add('navigateLaneMarker', NavigateLaneMarker(control=control), 
                transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished lane marker grid search mission. Result: {}".format(res))

def QualiMission():
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('quali', Quali(control=control), 
            transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished quali mission. Result {}".format(res))


if __name__ == '__main__':
    rospy.init_node('mission_planner',log_level=rospy.DEBUG)
    rospy.on_shutdown(endMission)

    control = Controller()

    # ----- UNCOMMENT BELOW TO RUN MISSION(S) -----
    #testRotationsMission()
    #laneMarkerGridSearchMission()
