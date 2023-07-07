#!/usr/bin/env python3

import rospy
import smach

from substates.breadth_first_search import *
from substates.in_place_search import *
from substates.linear_search import *
from substates.navigate_lane_marker import *
from substates.utility.controller import Controller
from substates.utility.state import StateTracker
from substates.utility.vision import *
from substates.quali import *
from substates.quaternion_test import *
from substates.trick import *
from substates.navigate_gate import *
from substates.quali_quaternion import *
from substates.navigate_buoy import *
from substates.octagon_task import *


def endMission(msg):
    print(msg)
    control.stop_in_place()

def endPlanner(msg="Shutting down mission planner."):
    print(msg)
    control.kill()

def QualiMission():
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('quali', Quali(control=control), 
            transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished quali mission. Result {}".format(res))
    
def QualiQuaternionMission():
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('quali', QualiQuaternion(control=control), 
            transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished quali mission. Result {}".format(res))
    
def QuaternionTestMission():
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('quaternion', QuaternionTest(control=control), 
            transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished quaternion mission. Result {}".format(res))

def tricks(t):
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('trick', Trick(control=control, trick_type=t), 
        transitions={'success': 'success', 'failure':'failure'})
        res = sm.execute()
    endMission("Finished trick. Result {}".format(res))

def master_planner():
    control.moveDelta((1, 0, -0.5))
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('find_gate', InPlaceSearch(timeout=120, target_class=global_class_ids["Gate"], min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_gate_no_go_through', 'failure': 'failure'})
        
        smach.StateMachine.add('navigate_gate_no_go_through', NavigateGate(control=control, mapping=mapping, state=state, goThrough=False, target_symbol=target_symbol, gate_class=global_class_ids["Gate"]), 
                transitions={'success': 'tricks', 'failure': 'failure'})
        
        smach.StateMachine.add('tricks', Trick(control=control, trick_type="roll"), 
                transitions={'success': 'navigate_gate_go_through', 'failure': 'failure'})
        
        smach.StateMachine.add('navigate_gate_go_through', NavigateGate(control=control, mapping=mapping, state=state, goThrough=True, target_symbol=target_symbol, gate_class=global_class_ids["Gate"]), 
                transitions={'success': 'find_lane_marker', 'failure': 'failure'})
        
        smach.StateMachine.add('find_lane_marker', BreadthFirstSearch(timeout=120, expansionAmt=1, target_class=global_class_ids["Lane Marker"], min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_lane_marker', 'failure': 'failure'})

        smach.StateMachine.add('navigate_lane_marker', NavigateLaneMarker(origin_class=1, control=control, mapping=mapping, state=state, lane_marker_class=global_class_ids["Lane Marker"]), 
                transitions={'success': 'find_buoy', 'failure': 'failure'})
        
        smach.StateMachine.add('find_buoy', LinearSearch(timeout=120, forward_speed=10, target_class=global_class_ids["Buoy"], min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_buoy', 'failure': 'failure'})

        smach.StateMachine.add('navigate_buoy', NavigateBuoy(control=control, mapping=mapping, state=state, buoy_class=global_class_ids["Buoy"], target_symbol_class=global_class_ids[target_symbol]), 
                transitions={'success': 'go_near_octagon', 'failure':'failure'})
        
        smach.StateMachine.add('go_near_octagon', GoToOctagon(control=control, search_point=octagon_approximate_location),
                transitions={'success': 'find_octagon', 'failure':'failure'})
        
        smach.StateMachine.add('find_octagon', BreadthFirstSearch(timeout=120, expansionAmt=1, target_class=global_class_ids["Octagon"], min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_octagon', 'failure':'failure'})
        
        smach.StateMachine.add('navigate_octagon', NavigateOctagon(control=control, mapping=mapping, state=state, octagon_class=global_class_ids["Octagon"]), 
                transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished Robosub with result: " + str(res) + "!!!!!!!!!")

octagon_approximate_location = (5,5)
global_class_ids = {"Lane Marker":0, "Gate":1, "Buoy":2, "Octagon Table":3, "Earth Symbol":4, "Abydos Symbol":5, "Octagon":6}


if __name__ == '__main__':
    rospy.init_node('mission_planner',log_level=rospy.DEBUG)
    rospy.on_shutdown(endPlanner)

    try:
        mapping = ObjectMapper()
        state = StateTracker()
        control = Controller(rospy.Time(0))
        target_symbol = "Earth Symbol" # "Abydos Symbol"

        master_planner()

        # ----- UNCOMMENT BELOW TO RUN MISSION(S) -----
        #testRotationsMission()
        #laneMarkerGridSearchMission()
    except KeyboardInterrupt:
        endPlanner("Mission end prompted by user. Killing.")
        exit()
