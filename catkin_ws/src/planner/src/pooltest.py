#!/usr/bin/env python3

import rospy
import smach

# TODO - simplify? uncouple with mission
from mission import *

if __name__ == "__main__":
    rospy.init_node('pooltest_planner')

    # these are required objects by some of the states
    mapping = ObjectMapper()
    state = StateTracker() #TODO - why should a state need to know something else about state info?
    control = Controller(rospy.Time(0))

    options = [
            '[0] breadth first search (lane marker)', # TODO - searching for a particular object isn't a great 'mission'
            '[1] in-place search (lane marker)', 
            '[2] linear search (buoy)', 
            '[3] quali', 
            '[4] quali - quaternion', 
            '[5] gate task', 
            '[6] trick', 
            '[7] lane marker', 
            '[8] buoy task', 
            '[9] octagon task', 
            ]
    
    target_symbol = "Earth Symbol" # "Abydos Symbol"

    for option in options:
        print(option)

    menu_index = input()

    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        # breadth first search
        if menu_index == '0':
            smach.StateMachine.add('breadth_first_search', 
                BreadthFirstSearch(timeout=120, expansionAmt=1, target_class="Lane Marker", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'success', 'failure':'failure'})

        # in-place search
        elif menu_index == '1':
            smach.StateMachine.add('in_place_search', 
                InPlaceSearch(timeout=120, target_class="Lane Marker", min_objects=1, control=control, mapping=mapping), 
                transitions={'success': 'success', 'failure':'failure'})

        # linear search
        elif menu_index == '2':
            smach.StateMachine.add('linear_search', 
                LinearSearch(timeout=120, forward_speed=10, target_class="Buoy", min_objects=1, control=control, mapping=mapping),
                transitions={'success': 'success', 'failure':'failure'})

        # quali
        elif menu_index == '3':
            smach.StateMachine.add('quali', 
                Quali(control=control),
                transitions={'success': 'success', 'failure':'failure'})

        # quali - quaternion
        elif menu_index == '4':
            smach.StateMachine.add('quali', 
                QualiQuaternion(control=control),
                transitions={'success': 'success', 'failure':'failure'})

        # gate task 
        elif menu_index == '5':
            smach.StateMachine.add('gate', 
                # TODO - add states to find/align to gate?
                NavigateGate(control=control, mapping=mapping, state=state, goThrough=True, target_symbol=target_symbol),
                transitions={'success': 'success', 'failure':'failure'})

        # trick 
        elif menu_index == '6':
            smach.StateMachine.add('trick', 
                Trick(control=control, trick_type='roll'),
                transitions={'success': 'success', 'failure':'failure'})

        # lane marker 
        elif menu_index == '7':
            smach.StateMachine.add('lane_marker', 
                # TODO - add states to find to lane marker?
                NavigateLaneMarker(origin_class="Gate", control=control, mapping=mapping, state=state),
                transitions={'success': 'success', 'failure':'failure'})

        # buoy task 
        elif menu_index == '8':
            smach.StateMachine.add('buoy', 
                # TODO - add states to find buoy?
                NavigateBuoy(control=control, mapping=mapping, state=state, target_symbol=target_symbol, buoy_width=1.2, buoy_height=1.2),
                transitions={'success': 'success', 'failure':'failure'})

        # octagon 
        elif menu_index == '9':
            smach.StateMachine.add('octagon', 
                # TODO - add states to find octagon?
                NavigateOctagon(control=control, mapping=mapping, state=state),
                transitions={'success': 'success', 'failure':'failure'})

        else: 
            exit()
    try:
        res = sm.execute()
    except KeyboardInterrupt:
        if sm is not None: sm.request_preempt()
    endMission("Finished mission. Result {}".format(res))

