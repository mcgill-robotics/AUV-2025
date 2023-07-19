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
from substates.navigate_gate import *
from substates.quali_quaternion import *
from substates.quali_vision import *
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
    
def QualiVisionMission(isCompetition=False):
    if isCompetition: rospy.sleep(60)
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('find_quali_gate', LinearSearch(timeout=120, forward_speed=3, target_class="Quali Gate", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'quali', 'failure': 'failure'})
        smach.StateMachine.add('quali', QualiVision(control=control, mapping=mapping, state=state, quali_gate_width=quali_gate_width), 
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
        smach.StateMachine.add('find_gate', InPlaceSearch(timeout=120, target_class="Gate", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_gate_no_go_through', 'failure': 'failure'})
        
        smach.StateMachine.add('navigate_gate_no_go_through', NavigateGate(control=control, mapping=mapping, state=state, goThrough=False, target_symbol=target_symbol, gate_width=gate_width), 
                transitions={'success': 'tricks', 'failure': 'failure'})
        
        smach.StateMachine.add('tricks', Trick(control=control, trick_type="yaw", num_full_spins=2), 
                transitions={'success': 'navigate_gate_go_through', 'failure': 'failure'})
        
        smach.StateMachine.add('navigate_gate_go_through', NavigateGate(control=control, mapping=mapping, state=state, goThrough=True, target_symbol=target_symbol, gate_width=gate_width), 
                transitions={'success': 'find_lane_marker', 'failure': 'failure'})
        
        smach.StateMachine.add('find_lane_marker', BreadthFirstSearch(timeout=120, expansionAmt=1, target_class="Lane Marker", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_lane_marker', 'failure': 'failure'})

        smach.StateMachine.add('navigate_lane_marker', NavigateLaneMarker(origin_class="Gate", control=control, mapping=mapping, state=state), 
                transitions={'success': 'find_buoy', 'failure': 'failure'})

        smach.StateMachine.add('find_buoy', LinearSearch(timeout=120, forward_speed=5, target_class="Buoy", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_buoy', 'failure': 'failure'})

        smach.StateMachine.add('navigate_buoy', NavigateBuoy(control=control, mapping=mapping, state=state, target_symbol=target_symbol), 
                transitions={'success': 'go_near_octagon', 'failure':'failure'})
        
        smach.StateMachine.add('go_near_octagon', GoToOctagon(control=control, search_point=octagon_approximate_location),
                transitions={'success': 'find_octagon', 'failure':'failure'})

        smach.StateMachine.add('find_octagon', BreadthFirstSearch(timeout=120, expansionAmt=1, target_class="Octagon Table", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'navigate_octagon', 'failure':'failure'})
        
        smach.StateMachine.add('navigate_octagon', NavigateOctagon(control=control, mapping=mapping, state=state), 
                transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endPlanner("Finished Robosub with result: " + str(res) + "!!!!!!!!!")

octagon_approximate_location = (5,5) # [COMP] UPDATE WITH ACTUAL SEARCH POINT FOR OCTAGON
quali_gate_width = 2 # [COMP] update with actual width in meters
gate_width = 3
target_symbol = "Earth Symbol" # "Abydos Symbol"

if __name__ == '__main__':
    rospy.init_node('mission_planner',log_level=rospy.DEBUG)
    rospy.on_shutdown(endPlanner)

    try:
        mapping = ObjectMapper()
        state = StateTracker()
        control = Controller(rospy.Time(0))
        sm = None
        
        # [COMP] MAKE SURE AUV IS IN COORDINATE FRAME WHERE OCTAGON LOCATION WAS MEASURED
        pub_DVL = rospy.Publisher('/reset_state_planar', Empty, queue_size=1)
        pub_DVL.publish(Empty())

        #rospy.sleep(60) # [COMP] UNCOMMENT

        control.move((None,None,-1), callback=lambda a,b: None)
        control.moveDelta((0,0,0), callback=lambda a,b: None)
        control.rotateEuler((0,0,None))


        # while not rospy.is_shutdown():
        #     control.rotateEuler((0,0,0))
        #     control.rotateEuler((0,0,90))
        #     control.rotateEuler((0,90,90))
        #     control.rotateEuler((-90,0,0))
        #BuoysMission()  

        sm = smach.StateMachine(outcomes=['success', 'failure']) 
        with sm:
            smach.StateMachine.add('go_near_octagon', GoToOctagon(control=control, search_point=octagon_approximate_location),
                    transitions={'success': 'find_octagon', 'failure':'failure'})

            smach.StateMachine.add('find_octagon', BreadthFirstSearch(timeout=120, expansionAmt=1, target_class=global_class_ids["Octagon"], min_objects=1, control=control, mapping=mapping), 
                    transitions={'success': 'navigate_octagon', 'failure':'failure'})
            
            smach.StateMachine.add('navigate_octagon', NavigateOctagon(control=control, mapping=mapping, state=state, octagon_class=global_class_ids["Octagon"]), 
                    transitions={'success': 'success', 'failure':'failure'})
            res = sm.execute()

        # ----- UNCOMMENT BELOW TO RUN MISSION(S) -----
        #testRotationsMission()
        #laneMarkerGridSearchMission()
    except KeyboardInterrupt:
        #ASSUMING ONE CURRENTLY RUNNING STATE MACHINE AT A TIME
        if sm is not None: sm.request_preempt()
    finally:
        endPlanner()
