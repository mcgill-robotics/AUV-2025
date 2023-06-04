#!/usr/bin/env python3

import rospy
import smach

from substates.breadth_first_search import *
from substates.grid_search import *
from substates.linear_search import *
from substates.navigate_lane_marker import *
from substates.test_submerged_rotations import *
from substates.utility.controller import Controller
from substates.utility.state import StateTracker
from substates.utility.vision import *
from substates.quali import *
from substates.trick import *

def endMission(msg="Shutting down mission planner."):
    print(msg)
    control.kill()

def testRotationsMission():
    control.moveDelta((0, 0, -2))
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('test_submerged_rotations', TestSubmergedRotations(hold_time = 5.0, control=control), 
                transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished rotation test mission.")

def laneMarkerGridSearchMission():
    control.moveDelta((0, 0, -0.5))
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('gridsearch', GridSearch(timeout=60, target_classes=[0, 1], control=control, mapping=mapping), 
                transitions={'success': 'navigateLaneMarker', 'failure':'failure'})
        smach.StateMachine.add('navigateLaneMarker', NavigateLaneMarker(control=control, mapping=mapping, state=state), 
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

def Tricks(t):
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        if t == "roll":
            smach.StateMachine.add('roll', Tricks(control=control), 
            transitions={'success': 'success', 'failure':'failure'})
            res = sm.execute_roll()
        elif t == "pitch":
            smach.StateMachine.add('pitch', Tricks(control=control), 
            transitions={'success': 'success', 'failure':'failure'})
            res = sm.execute_pitch()
        elif t == "yaw":
            smach.StateMachine.add('yaw', Tricks(control=control), 
            transitions={'success': 'success', 'failure':'failure'})
            res = sm.execute_yaw()
        else:
            res = "trick not identified"
    endMission("Finished trick. Result {}".format(res))


if __name__ == '__main__':
    rospy.init_node('mission_planner',log_level=rospy.DEBUG)
    rospy.on_shutdown(endMission)

    control = Controller()
    state = StateTracker()
    mapping = ObjectMapper()
    Tricks(t="roll")


    # ----- UNCOMMENT BELOW TO RUN MISSION(S) -----
    #testRotationsMission()
    #laneMarkerGridSearchMission()
