#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import String

from substates.breadth_first_search import *
from substates.in_place_search import *
from substates.linear_search import *
from substates.navigate_lane_marker import *
from substates.utility.controller import *
from substates.utility.state import *
from substates.utility.vision import *
from substates.trick import *
from substates.navigate_gate import *
from substates.navigate_buoy import *
from substates.octagon_task import *
from substates.navigate_pinger import *


def endMission(msg):
    print(msg)
    control.freeze_pose()

def endPlanner(msg="Shutting down mission planner."):
    pub_mission_display.publish("Exit")
    print(msg)
    control.kill()

def gateMission():
    pub_mission_display.publish("Gate")
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('find_gate', InPlaceSearch(timeout=120, target_class="Gate", min_objects=1, control=control, mapping=mapping, search_depth=-2), 
            transitions={'success': 'navigate_gate_go_through', 'failure': 'failure'})
        smach.StateMachine.add('navigate_gate_go_through', NavigateGate(control=control, mapping=mapping, state=state, goThrough=True, gate_width=gate_width), 
            transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    # display_mission.updateMission("Gate Task {}".format(res))
    endMission("Finished gate mission. Result {}".format(res))

def buoyMission():
    pub_mission_display.publish("Buoy")
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    with sm:
        smach.StateMachine.add('find_buoy', InPlaceSearch(timeout=120, target_class="Buoy", min_objects=1, control=control, mapping=mapping, search_depth=-2), 
                transitions={'success': 'navigate_buoy', 'failure': 'failure'})

        smach.StateMachine.add('navigate_buoy', NavigateBuoy(control=control, mapping=mapping, state=state, target_symbol=target_symbol, buoy_width=buoy_width, buoy_height=buoy_height), 
                transitions={'success': 'success', 'failure':'failure'})
        
    res = sm.execute()
    # display_mission.updateMission("Buoy Task {}".format(res))
    endMission("Finished buoy mission. Result {}".format(res))

def tricks(t):
    pub_mission_display.publish("Tricks")
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('trick', Trick(control=control, trick_type=t, state=state, num_full_spins=3), 
        transitions={'success': 'success', 'failure':'failure'})
        res = sm.execute()
    # display_mission.updateMission("Tricks {}".format(res))
    endMission("Finished trick. Result {}".format(res))

def laneMarkerMission():
    pub_mission_display.publish("Lane")
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    with sm:
        smach.StateMachine.add('find_lane_marker', BreadthFirstSearch(timeout=120, expansionAmt=0.5, target_class="Lane Marker", min_objects=1, control=control, mapping=mapping, search_depth=-1), 
                transitions={'success': 'navigate_lane_marker', 'failure': 'failure'})

        smach.StateMachine.add('navigate_lane_marker', NavigateLaneMarker(origin_class="", control=control, mapping=mapping, state=state), 
                transitions={'success': 'success', 'failure': 'failure'})
        res = sm.execute()
    # display_mission.updateMission("Lane Marker {}".format(res))
    endMission("Finished lane marker. Result {}".format(res))
    
def pingerMission():
        pub_mission_display.publish("Pinger")
        global sm
        sm = smach.StateMachine(outcomes=['success', 'failure'])
        with sm:
                # Turn towards pinger bearing and move towards it until you find an object
                smach.StateMachine.add('navigate_pinger', GoToPinger(control=control, mapping=mapping, state=state, pinger_num=4), 
                        transitions={'success': 'success', 'failure':'failure', 'search': 'breadth_first_search'})
                # TODO [COMP]: Specify target class
                smach.StateMachine.add('breadth_first_search', BreadthFirstSearch(timeout=120, target_class="", expansionAmt=0.5, min_objects=1, control=control, mapping=mapping, search_depth=-1), 
                        transitions={'success': 'success', 'failure':'failure'})
                res = sm.execute()
        # display_mission.updateMission("Pinger {}".format(res))
        endMission("Finished pinger. Result {}".format(res))

def semiFinals():
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('find_gate', InPlaceSearch(timeout=120, target_class="Gate", min_objects=1, control=control, mapping=mapping, search_depth=-2), 
                transitions={'success': 'navigate_gate_no_go_through', 'failure': 'failure'})
        
        smach.StateMachine.add('navigate_gate_no_go_through', NavigateGate(control=control, mapping=mapping, state=state, goThrough=False, target_symbol=target_symbol, gate_width=gate_width), 
                transitions={'success': 'tricks', 'failure': 'failure'})
        
        smach.StateMachine.add('tricks', Trick(control=control, trick_type="yaw", num_full_spins=2), 
                transitions={'success': 'navigate_gate_go_through', 'failure': 'failure'})
        
        smach.StateMachine.add('navigate_gate_go_through', NavigateGate(control=control, mapping=mapping, state=state, goThrough=True, target_symbol=target_symbol, gate_width=gate_width), 
                transitions={'success': 'find_lane_marker', 'failure': 'failure'})
        
        smach.StateMachine.add('find_lane_marker', BreadthFirstSearch(timeout=120, expansionAmt=0.5, target_class="Lane Marker", min_objects=1, control=control, mapping=mapping, search_depth=-1), 
                transitions={'success': 'navigate_lane_marker', 'failure': 'failure'})

        smach.StateMachine.add('navigate_lane_marker', NavigateLaneMarker(origin_class="Gate", control=control, mapping=mapping, state=state), 
                transitions={'success': 'find_buoy', 'failure': 'failure'})

        smach.StateMachine.add('find_buoy', LinearSearch(timeout=120, forward_speed=5, target_class="Buoy", min_objects=1, control=control, mapping=mapping, search_depth=-2), 
                transitions={'success': 'navigate_buoy', 'failure': 'failure'})

        smach.StateMachine.add('navigate_buoy', NavigateBuoy(control=control, mapping=mapping, state=state, target_symbol=target_symbol, buoy_width=buoy_width, buoy_height=buoy_height), 
                transitions={'success': 'find_second_lane_marker', 'failure':'failure'})
        
        smach.StateMachine.add('find_second_lane_marker', BreadthFirstSearch(timeout=120, expansionAmt=0.5, target_class="Lane Marker", min_objects=2, control=control, mapping=mapping, search_depth=-1), 
                transitions={'success': 'navigate_second_lane_marker', 'failure': 'failure'})

        smach.StateMachine.add('navigate_second_lane_marker', NavigateLaneMarker(origin_class="Buoy", control=control, mapping=mapping, state=state), 
                transitions={'success': 'find_octagon', 'failure': 'failure'})
        
        smach.StateMachine.add('find_octagon', LinearSearch(timeout=120, forward_speed=5, target_class="Octagon Table", min_objects=1, control=control, mapping=mapping, search_depth=-2), 
                transitions={'success': 'navigate_octagon', 'failure':'failure'})
        
        smach.StateMachine.add('navigate_octagon', NavigateOctagon(control=control, mapping=mapping, state=state), 
                transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endPlanner("Finished Robosub with result: " + str(res) + "!!!!!!!!!")

buoy_width = 1.2
buoy_height = 1.2
gate_width = 3
wait_time_for_comp = 30 # [COMP] make sure this is long enough

if __name__ == '__main__':
    mission = input("Enter mission to run: ")
    rospy.init_node('mission_planner',log_level=rospy.DEBUG)
    rospy.on_shutdown(endPlanner)

    pub_mission_display = rospy.Publisher("/mission_display", String, queue_size=1)

    try:
        mapping = ObjectMapper()
        state = StateTracker()
        control = Controller(rospy.Time(0))
        pinger_num = 1 # [COMP] change this to an integer depending on which pinger is being used
        sm = None

        # Run a planner mission based on the user input 
        if mission == "tricks":
             tricks("roll")
        elif mission == "pinger":
             pingerMission()
        elif mission == "laneMarker":
             laneMarkerMission()
        elif mission == "buoy":
             buoyMission()
        elif mission == "gate":
             gateMission()
        else: 
             print("Invalid mission name")

    except KeyboardInterrupt:
        #ASSUMING ONE CURRENTLY RUNNING STATE MACHINE AT A TIME (NO THREADS)
        if sm is not None: sm.request_preempt()
    finally:
        endPlanner()