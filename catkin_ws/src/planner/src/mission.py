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
from substates.quali import *
from substates.trick import *
from substates.trick_effort import *
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
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('quali', Quali(control=control), 
            transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished quali mission. Result {}".format(res))
    
def QualiQuaternionMission():
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('quali', QualiQuaternion(control=control), 
            transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished quali mission. Result {}".format(res))
    
def GateMission():
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('find_gate', InPlaceSearch(timeout=120, target_class="Gate", min_objects=1, control=control, mapping=mapping), 
            transitions={'success': 'navigate_gate_go_through', 'failure': 'failure'})
        smach.StateMachine.add('navigate_gate_go_through', NavigateGate(control=control, mapping=mapping, state=state, goThrough=True, target_symbol=target_symbol), 
            transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished gate mission. Result {}".format(res))
    
def BuoysMission():
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    with sm:
        smach.StateMachine.add('find_buoy', InPlaceSearch(timeout=120, target_class="Buoy", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_buoy', 'failure': 'failure'})

        smach.StateMachine.add('navigate_buoy', NavigateBuoy(control=control, mapping=mapping, state=state, target_symbol=target_symbol), 
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

def tricks_effort(t):
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('trick', TrickEffort(control=control, trick_type=t, effort=10), 
        transitions={'success': 'success', 'failure':'failure'})
        res = sm.execute()
    endMission("Finished trick. Result {}".format(res))
    
def laneMarkerMission():
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    with sm:
        smach.StateMachine.add('find_lane_marker', BreadthFirstSearch(timeout=120, expansionAmt=1, target_class="Lane Marker", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_lane_marker', 'failure': 'failure'})

        smach.StateMachine.add('navigate_lane_marker', NavigateLaneMarker(origin_class="", control=control, mapping=mapping, state=state), 
                transitions={'success': 'success', 'failure': 'failure'})
        res = sm.execute()
    endMission("Finished lane marker. Result {}".format(res))
        

def master_planner():
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        # NO BIG BUGS
        # MAYBE ADD CHANGE DEPTH IF NOT FINDING OBJECT
        smach.StateMachine.add('find_gate', InPlaceSearch(timeout=120, target_class="Gate", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_gate_no_go_through', 'failure': 'failure'})
        
        # NO BIG BUGS!
        smach.StateMachine.add('navigate_gate_no_go_through', NavigateGate(control=control, mapping=mapping, state=state, goThrough=False, target_symbol=target_symbol), 
                transitions={'success': 'tricks', 'failure': 'failure'})
        
        # UNTESTED
        smach.StateMachine.add('tricks', Trick(control=control, trick_type="roll"), 
                transitions={'success': 'navigate_gate_go_through', 'failure': 'failure'})
        
        # NO BIG BUGS - RETEST AFTER FIXING VISION
        smach.StateMachine.add('navigate_gate_go_through', NavigateGate(control=control, mapping=mapping, state=state, goThrough=True, target_symbol=target_symbol), 
                transitions={'success': 'find_lane_marker', 'failure': 'failure'})
        
        # NO BIG BUGS!
        smach.StateMachine.add('find_lane_marker', BreadthFirstSearch(timeout=120, expansionAmt=1, target_class="Lane Marker", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_lane_marker', 'failure': 'failure'})

        # GOES TO WRONG HEADING
        smach.StateMachine.add('navigate_lane_marker', NavigateLaneMarker(origin_class="Gate", control=control, mapping=mapping, state=state), 
                transitions={'success': 'find_buoy', 'failure': 'failure'})
           
        # CHANGE LINEAR SEARCH TO STEPS INSTEAD OF CONSTANT SPEED
        # TEST WITH MISSION PLANNER
        # OTHER THAN THAT NO BIG BUGS!
        smach.StateMachine.add('find_buoy', LinearSearch(timeout=120, forward_speed=10, target_class="Buoy", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_buoy', 'failure': 'failure'})

        # TEST WHEN BUOY CAN BE DETECTED RELIABLY
        smach.StateMachine.add('navigate_buoy', NavigateBuoy(control=control, mapping=mapping, state=state, target_symbol=target_symbol), 
                transitions={'success': 'go_near_octagon', 'failure':'failure'})
        
        smach.StateMachine.add('go_near_octagon', GoToOctagon(control=control, search_point=octagon_approximate_location),
                transitions={'success': 'find_octagon', 'failure':'failure'})
        
        # NO BIG BUGS!
        smach.StateMachine.add('find_octagon', BreadthFirstSearch(timeout=120, expansionAmt=1, target_class="Octagon Table", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_octagon', 'failure':'failure'})
        
        smach.StateMachine.add('navigate_octagon', NavigateOctagon(control=control, mapping=mapping, state=state), 
                transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endPlanner("Finished Robosub with result: " + str(res) + "!!!!!!!!!")

octagon_approximate_location = (5,5)


if __name__ == '__main__':
    rospy.init_node('mission_planner',log_level=rospy.DEBUG)
    rospy.on_shutdown(endPlanner)

    try:
        mapping = ObjectMapper()
        state = StateTracker()
        control = Controller(rospy.Time(0))
        target_symbol = "Earth Symbol" # "Abydos Symbol"
        sm = None
        
        #SINCE I CANT TEST MYSELF UNTIL I HAVE TUNED CUSTOM PIDs FOR MY COMPUTER THIS IS WHAT NEEDS TESTING FOR REFERENCE (RECENT CHANGES):
            # - rotate delta
            # - tricks effort mission
            # - tricks mission
            # - quali mission
            # - quali quaternion mission
            # - in place search depth changes
            # - navigate lane marker

        BuoysMission()  

        # get mission to run from command line argument
        # TODO - this is a bit hackish but probably fine
        # mission = sys.argv[1] 
        # if mission.startswith("__mission"):
        #     mission = mission.replace("__mission:=", "")
        # else:
        #     mission = None


        # print("mission", mission)
        # if mission is None:
        #     master_planner()
        # elif mission == "quali":
        #     QualiMission()
        # else:
        #     raise Exception("invalid mission")

        # ----- UNCOMMENT BELOW TO RUN MISSION(S) -----
        #testRotationsMission()
        #laneMarkerGridSearchMission()
    except KeyboardInterrupt:
        #ASSUMING ONE CURRENTLY RUNNING STATE MACHINE AT A TIME
        if sm is not None: sm.request_preempt()
        endPlanner("Mission end prompted by user. Killing.")
        exit()
