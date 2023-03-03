#state machine used by AUV when it sees a lane marker on downward camera
#assumes AUV is in nominal orientation
#assumes AUV is not facing direction from where it came (i.e. that lane marker heading most similar to its own direction is not where it should go)
#uses only camera feed to adjust itself to be directly above the lane marker
#uses the camera feed to rotate itself to the lane marker outgoing heading
#done

#!/usr/bin/env python3

import rospy
import smach

from states import *

if __name__ == '__main__':
    rospy.init_node('mission_planner')
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('submerge', DepthState(4.0), 
                transitions={'success': 'success', 'failure':'failure'})

    res = sm.execute()
