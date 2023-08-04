#!/usr/bin/env python3

import rospy
import smach

from substates.breadth_first_search import *
from substates.in_place_search import *
from substates.linear_search import *
from substates.navigate_lane_marker import *
from substates.utility.controller import *
from substates.utility.state import *
from substates.utility.vision import *
from substates.trick import *
from substates.navigate_gate import *
from substates.quali_vision import *
from substates.navigate_buoy import *
from substates.octagon_task import *
from substates.initialize_for_comp import *


def endMission(msg):
    print(msg)
    control.stop_in_place()

def endPlanner(msg="Shutting down mission planner."):
    print(msg)
    control.kill()

def qualiVisionMission():
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('initialization', InitializeForComp(wait_time=wait_time_for_comp), 
                transitions={'success': 'find_quali'})
        smach.StateMachine.add('find_quali', LinearSearch(timeout=120, forward_speed=5, target_class="Quali Gate", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'quali', 'failure': 'failure'})
        smach.StateMachine.add('quali', QualiVision(control=control, mapping=mapping, state=state, quali_gate_width=quali_gate_width), 
            transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished quali mission. Result {}".format(res))

def gateMission():
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('find_gate', InPlaceSearch(timeout=120, target_class="Gate", min_objects=1, control=control, mapping=mapping), 
            transitions={'success': 'navigate_gate_go_through', 'failure': 'failure'})
        smach.StateMachine.add('navigate_gate_go_through', NavigateGate(control=control, mapping=mapping, state=state, goThrough=True, target_symbol=target_symbol, gate_width=gate_width), 
            transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished gate mission. Result {}".format(res))

def buoyMission():
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    with sm:
        smach.StateMachine.add('find_buoy', InPlaceSearch(timeout=120, target_class="Buoy", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_buoy', 'failure': 'failure'})

        smach.StateMachine.add('navigate_buoy', NavigateBuoy(control=control, mapping=mapping, state=state, target_symbol=target_symbol, buoy_width=buoy_width, buoy_height=buoy_height), 
                transitions={'success': 'success', 'failure':'failure'})
        
    res = sm.execute()
    endMission("Finished buoy mission. Result {}".format(res))

def tricks(t):
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('trick', Trick(control=control, trick_type=t), 
        transitions={'success': 'success', 'failure':'failure'})
        res = sm.execute()
    endMission("Finished trick. Result {}".format(res))

def laneMarkerMission():
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    with sm:
        smach.StateMachine.add('find_lane_marker', BreadthFirstSearch(timeout=120, expansionAmt=0.5, target_class="Lane Marker", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_lane_marker', 'failure': 'failure'})

        smach.StateMachine.add('navigate_lane_marker', NavigateLaneMarker(origin_class="", control=control, mapping=mapping, state=state), 
                transitions={'success': 'success', 'failure': 'failure'})
        res = sm.execute()
    endMission("Finished lane marker. Result {}".format(res))

def semiFinals():
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('initialization', InitializeForComp(wait_time=wait_time_for_comp), 
                transitions={'success': 'find_gate'})
        
        smach.StateMachine.add('find_gate', InPlaceSearch(timeout=120, target_class="Gate", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_gate_no_go_through', 'failure': 'failure'})
        
        smach.StateMachine.add('navigate_gate_no_go_through', NavigateGate(control=control, mapping=mapping, state=state, goThrough=False, target_symbol=target_symbol, gate_width=gate_width), 
                transitions={'success': 'tricks', 'failure': 'failure'})
        
        smach.StateMachine.add('tricks', Trick(control=control, trick_type="yaw", num_full_spins=2), 
                transitions={'success': 'navigate_gate_go_through', 'failure': 'failure'})
        
        smach.StateMachine.add('navigate_gate_go_through', NavigateGate(control=control, mapping=mapping, state=state, goThrough=True, target_symbol=target_symbol, gate_width=gate_width), 
                transitions={'success': 'find_lane_marker', 'failure': 'failure'})
        
        smach.StateMachine.add('find_lane_marker', BreadthFirstSearch(timeout=120, expansionAmt=0.5, target_class="Lane Marker", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_lane_marker', 'failure': 'failure'})

        smach.StateMachine.add('navigate_lane_marker', NavigateLaneMarker(origin_class="Gate", control=control, mapping=mapping, state=state), 
                transitions={'success': 'find_buoy', 'failure': 'failure'})

        smach.StateMachine.add('find_buoy', LinearSearch(timeout=120, forward_speed=5, target_class="Buoy", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_buoy', 'failure': 'failure'})

        smach.StateMachine.add('navigate_buoy', NavigateBuoy(control=control, mapping=mapping, state=state, target_symbol=target_symbol, buoy_width=buoy_width, buoy_height=buoy_height), 
                transitions={'success': 'find_second_lane_marker', 'failure':'failure'})
        
        smach.StateMachine.add('find_second_lane_marker', BreadthFirstSearch(timeout=120, expansionAmt=0.5, target_class="Lane Marker", min_objects=2, control=control, mapping=mapping), 
                transitions={'success': 'navigate_second_lane_marker', 'failure': 'failure'})

        smach.StateMachine.add('navigate_second_lane_marker', NavigateLaneMarker(origin_class="Buoy", control=control, mapping=mapping, state=state), 
                transitions={'success': 'find_octagon', 'failure': 'failure'})
        
        smach.StateMachine.add('find_octagon', LinearSearch(timeout=120, forward_speed=5, target_class="Octagon Table", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_octagon', 'failure':'failure'})
        
        smach.StateMachine.add('navigate_octagon', NavigateOctagon(control=control, mapping=mapping, state=state), 
                transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endPlanner("Finished Robosub with result: " + str(res) + "!!!!!!!!!")

buoy_width = 1.2
buoy_height = 1.2
gate_width = 3
target_symbol = "Earth Symbol" # "Abydos Symbol"
wait_time_for_comp = 30 # [COMP] make sure this is long enough

if __name__ == '__main__':
    rospy.init_node('mission_planner',log_level=rospy.DEBUG)
    rospy.on_shutdown(endPlanner)

    try:
        mapping = ObjectMapper()
        state = StateTracker()
        control = Controller(rospy.Time(0))
        sm = None


        control.move((None,None,-1), callback=lambda a,b: None)
        control.moveDelta((0,0,0), callback=lambda a,b: None)
        control.rotateEuler((0,0,None))
        # while True:
        #     control.rotateEuler((0,0,0))
        #     control.rotateEuler((0,0,90))
        #     control.rotateEuler((0,90,90))
        #     control.rotateEuler((-90,0,0))
            

        # ----- UNCOMMENT BELOW TO RUN MISSION(S) -----
        #gateMission()
        #qualiVisionMission()
        #buoyMission()  
        #tricks()  
        #laneMarkerGridSearchMission()
    except KeyboardInterrupt:
        #ASSUMING ONE CURRENTLY RUNNING STATE MACHINE AT A TIME (NO THREADS)
        if sm is not None: sm.request_preempt()