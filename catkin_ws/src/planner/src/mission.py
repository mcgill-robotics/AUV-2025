#!/usr/bin/env python3

import rospy
import smach

from sub_states import *
from sub_states.utility import *

class descend(smach.State):
    def __init__(self, depth=-2.0):
        super().__init__(outcomes=['success'])
        self.depth = depth
        
    def execute(self, ud):
        descended = False
        def done():
            global descended
            descended = True
        controller.moveDelta((0, 0, self.depth), done)
        while not descended: rospy.sleep(0.1)
        return 'success'

def shutdown():
    controller.preemptCurrentAction()
    controller.velocity((0,0,0))
    controller.angularVelocity((0,0,0))

if __name__ == '__main__':
    rospy.init_node('mission_planner')
    rospy.on_shutdown(shutdown)
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('descend', descend(depth=-2.0), 
                transitions={'success': 'gridsearch'})
        smach.StateMachine.add('gridsearch', grid_search.GridSearch(target_class=0, timeout=60), 
                transitions={'success': 'navigateLaneMarker', 'failure':'failure'})
        smach.StateMachine.add('navigateLaneMarker', navigate_lane_marker.NavigateLaneMarker(target_class=0, timeout=60), 
                transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    print("Finished mission.")
    shutdown()
