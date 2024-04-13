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

def endMission(msg):
    print(msg)
    control.freeze_pose()

def endPlanner(msg="Shutting down mission planner."):
    pub_mission_display.publish("End")
    print(msg)
    control.kill()

def gateMission():
    pub_mission_display.publish("Gate")
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('find_gate', InPlaceSearch(control, mapping, target_class="Gate", min_objects=1), 
            transitions={'success': 'navigate_gate_go_through', 'failure': 'failure'})
        smach.StateMachine.add('navigate_gate_go_through', NavigateGate(control=control, mapping=mapping, state=state, goThrough=True, target_symbol=target_symbol, gate_width=gate_width), 
            transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished gate mission. Result {}".format(res))

def buoyMission():
    pub_mission_display.publish("Buoy")
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    with sm:
        smach.StateMachine.add('find_buoy', InPlaceSearch(control, mapping, target_class="Buoy", min_objects=1),
                transitions={'success': 'navigate_buoy', 'failure': 'failure'})

        smach.StateMachine.add('navigate_buoy', NavigateBuoy(control=control, mapping=mapping, state=state), 
                transitions={'success': 'success', 'failure':'failure'})
        
    res = sm.execute()
    endMission("Finished buoy mission. Result {}".format(res))

def tricks(t):
    pub_mission_display.publish("Tricks")
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('trick', Trick(control=control, trick_type=t), 
        transitions={'success': 'success', 'failure':'failure'})
        res = sm.execute()
    endMission("Finished trick. Result {}".format(res))

def laneMarkerMission():
    pub_mission_display.publish("Lane")
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    with sm:
        smach.StateMachine.add('find_lane_marker', BreadthFirstSearch(control, mapping, target_class="Lane Marker", min_objects=1), 
                transitions={'success': 'navigate_lane_marker', 'failure': 'failure'})

        smach.StateMachine.add('navigate_lane_marker', NavigateLaneMarker(control, mapping, state, origin_class=""), 
                transitions={'success': 'success', 'failure': 'failure'})
        res = sm.execute()
    endMission("Finished lane marker. Result {}".format(res))

def semiFinals():
    global sm
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('find_gate', InPlaceSearch(control, mapping, target_class="Gate", min_objects=1), 
                transitions={'success': 'navigate_gate_no_go_through', 'failure': 'failure'})
        
        smach.StateMachine.add('navigate_gate_no_go_through', NavigateGate(control=control, mapping=mapping, state=state, goThrough=False, target_symbol=target_symbol, gate_width=gate_width), 
                transitions={'success': 'tricks', 'failure': 'failure'})
        
        smach.StateMachine.add('tricks', Trick(control=control, trick_type="yaw", num_full_spins=2), 
                transitions={'success': 'navigate_gate_go_through', 'failure': 'failure'})
        
        smach.StateMachine.add('navigate_gate_go_through', NavigateGate(control=control, mapping=mapping, state=state, goThrough=True, target_symbol=target_symbol, gate_width=gate_width), 
                transitions={'success': 'find_lane_marker', 'failure': 'failure'})
        
        smach.StateMachine.add('find_lane_marker', BreadthFirstSearch(control, mapping, target_class="Lane Marker", min_objects=1), 
                transitions={'success': 'navigate_lane_marker', 'failure': 'failure'})

        smach.StateMachine.add('navigate_lane_marker', NavigateLaneMarker(control, mapping, state, origin_class="Gate"), 
                transitions={'success': 'find_buoy', 'failure': 'failure'})

        smach.StateMachine.add('find_buoy', LinearSearch(control, mapping, target_class="Buoy", min_objects=1), 
                transitions={'success': 'navigate_buoy', 'failure': 'failure'})

        smach.StateMachine.add('navigate_buoy', NavigateBuoy(control, mapping, state), 
                transitions={'success': 'find_second_lane_marker', 'failure':'failure'})
        
        smach.StateMachine.add('find_second_lane_marker', BreadthFirstSearch(control, mapping, target_class="Lane Marker", min_objects=2), 
                transitions={'success': 'navigate_second_lane_marker', 'failure': 'failure'})

        smach.StateMachine.add('navigate_second_lane_marker', NavigateLaneMarker(control, mapping, state, origin_class="Buoy"), 
                transitions={'success': 'find_octagon', 'failure': 'failure'})
        
        smach.StateMachine.add('find_octagon', LinearSearch(control, mapping, target_class="Octagon Table", min_objects=1), 
                transitions={'success': 'navigate_octagon', 'failure':'failure'})
        
        smach.StateMachine.add('navigate_octagon', NavigateOctagon(control=control, mapping=mapping, state=state), 
                transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endPlanner("Finished Robosub with result: " + str(res) + "!!!!!!!!!")



wait_time_for_comp = 30 # [COMP] make sure this is long enough

if __name__ == '__main__':
    rospy.init_node('mission_planner',log_level=rospy.DEBUG)
    rospy.on_shutdown(endPlanner)

    pub_mission_display = rospy.Publisher("/mission_display", String, queue_size=1)

    try:
        mapping = ObjectMapper()
        state = StateTracker()
        control = Controller(rospy.Time(0))
        sm = None

        competition()
        print("Waiting {} seconds before starting mission...".format(rospy.get_param("mission_wait_time")))
        rospy.sleep(rospy.get_param("mission_wait_time"))
        print("Starting mission.")

        # ----- UNCOMMENT BELOW TO RUN MISSION(S) -----
        # gateMission()
        #qualiVisionMission()
        #buoyMission()  
        #tricks()  
        # laneMarkerMission()
        buoyMission()
    except KeyboardInterrupt:
        #ASSUMING ONE CURRENTLY RUNNING STATE MACHINE AT A TIME (NO THREADS)
        if sm is not None: sm.request_preempt()
    finally:
        endPlanner()