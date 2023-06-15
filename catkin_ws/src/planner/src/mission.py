#!/usr/bin/env python3

import rospy
import smach

from substates.breadth_first_search import *
from substates.in_place_search import *
from substates.grid_search import *
from substates.linear_search import *
from substates.navigate_lane_marker import *
from substates.test_submerged_rotations import *
from substates.utility.controller import Controller
from substates.utility.state import StateTracker
from substates.utility.vision import *
from substates.quali import *
from substates.quaternion_test import *
from substates.trick import *
from substates.reposition import *
from substates.navigate_gate import *

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
        smach.StateMachine.add('gridsearch', GridSearch(timeout=60, target_classes=[(0, 1)], control=control, mapping=mapping), 
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

def QuaternionTestMission():
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('quaternion', QauternionTest(control=control), 
            transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished quaternion mission. Result {}".format(res))

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

def master_planner():
    control.moveDelta((0, 0, -0.5))
    control.rotateYaw(45) # Coin flip repositioning
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('find_gate', InPlaceSearch(timeout=9999, target_classes=[(1, 1)], control=control, mapping=mapping), 
                transitions={'success': 'navigate_gate', 'failure': 'reposition_find_gate'})

        # Add state machine to go backwards and heave and rotate (looking around) - if success, look again for gate
        smach.StateMachine.add('reposition_find_gate', RepositionGateBuoy(control=control),
                transitions={'success': 'find_gate', 'failure': 'failure'})
        
        smach.StateMachine.add('navigate_gate', NavigateGate(control=control, mapping=mapping, state=state), 
                transitions={'success': 'find_lane_marker', 'failure': 'failure'})
        smach.StateMachine.add('find_lane_marker', BreadthFirstSearch(target_classes=[(0, 1)], control=control, mapping=mapping), 
                transitions={'success': 'navigate_lane_marker', 'failure': 'reposition_find_lane_marker'})

        # Add state machine to heave (more vision agle)
        smach.StateMachine.add('reposition_find_lane_marker', RepositionLaneMarker(control=control),
                transitions={'success': 'find_lane_marker', 'failure': 'failure'})

        smach.StateMachine.add('navigate_lane_marker', NavigateLaneMarker(origin_class=1, control=control, mapping=mapping, state=state), 
                transitions={'success': 'find_buoy', 'failure': 'failure'})
        smach.StateMachine.add('find_buoy', LinearSearch(timeout=9999, forward_speed=5, target_classes=[(2,1)], control=control, mapping=mapping), 
                transitions={'success': 'navigate_buoy', 'failure': 'reposition_find_buoy'})

        # Add state machine to go backwards
        smach.StateMachine.add('reposition_find_buoy', RepositionGateBuoy(control=control),
                transitions={'success': 'find_buoy', 'failure': 'failure'})

        smach.StateMachine.add('navigate_buoy', NavigateBuoy(control=control, mapping=mapping, state=state), 
                transitions={'success': '??'})
    res = sm.execute()
    endMission("Finished Robosub!")


if __name__ == '__main__':
    rospy.init_node('mission_planner',log_level=rospy.DEBUG)
    rospy.on_shutdown(endMission)

    control = Controller(rospy.Time(0))
    QuaternionTestMission()


    # ----- UNCOMMENT BELOW TO RUN MISSION(S) -----
    #testRotationsMission()
    #laneMarkerGridSearchMission()
