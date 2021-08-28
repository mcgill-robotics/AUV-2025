#!/usr/bin/env python
import rospy
import roslaunch
import smach
import math
import actionlib
import time
import threading

from cv.msg import CvTarget
from std_msgs.msg import Bool, Float64, Float32MultiArray
from blinky.msg import TaskStatus
from geometry_msgs.msg import Vector3Stamped, Point
from threading import Thread
from planner.msg import LaneDetectorCenteringAction, LaneDetectorCenteringGoal,  LaneDetectorAlignmentAction, LaneDetectorAlignmentGoal

'''
I think we should put 2-3 and 4-5 in the same file because they belong to thew same task
'''
from Tasks.S1_GateTask            import GateTask
from Tasks.S2_GridSearch          import GridSearch
from Tasks.S3_LaneDetector        import LaneDetector
from Tasks.S4_NavigateToBuoy      import NavigateToBuoy
from Tasks.S5_TouchBuoyTask       import TouchBuoy
from Tasks.S6_NavigateToSurfacing import NavigateToSurfacing

# Navigation Target coordinates 
# X     : DVL
# Y     : DVL
# z     : Depth Senesor
# roll  : IMU
# Yaw   : IMU
# Pitch : IMU

def planner_all_tasks():
    rospy.init_node('mockMissionPlanner', anonymous=True)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['MissionSucceeded', 'MissionFailed', 'MissionPreempted'])

    # Open the container
    with sm:
        # Add states to the container
        # This is where the  transitions are defined!

        # Overall plan for state transitions
        # 1) Gate State - done
        # 2) Grid Search - done
        # 3) Lane Detector - done
        # 4) Surge To Buoy
        # 5) Buoy Task
        # 6) Grid Search - done
        # 7) Lane Detector - done
        # 8) Surge To Garlic
        # 9) Garlic Task
        # 10) Pinger To Torpedo
        # 11) Torpedo Task
        # 12) Pinger To Surface
        # 13) JOLLY GOOD
        
        # Gate task. Transitions Directly to pinger task...for now.
        smach.StateMachine.add('GateState', GateTask(), 
                                transitions={'gatePassed':'GridSearch',
                                            'gateMissed':'MissionFailed',
                                            'gatePreempted':'MissionPreempted'})
        
        smach.StateMachine.add('GridSearch', GridSearch(), 
                                transitions={'laneFound':'LaneDetectorForBuoy',
                                            'laneNotFound':'MissionFailed',
                                            'gridSearchPreempted':'MissionPreempted'})

        smach.StateMachine.add('LaneDetectorForBuoy',LaneDetector(),
                                transitions={'alignmentSuccess':'NavigateToBuoyTask',
                                            'alignmentFailure':'MissionFailed',
                                            'laneDetectorPreempted':'MissionPreempted'})

        smach.StateMachine.add('NavigateToBuoyTask',NavigateToBuoy(),
                                transitions={'buoyReached':'TouchBuoyTask',
                                            'buoyLost':'MissionFailed',
                                            'navigateToBuoyPreempted':'MissionPreempted'})

        smach.StateMachine.add('TouchBuoyTask',TouchBuoy(),
                                transitions={'touchedTheBuoy':'NavigateToSurfacingTask',
                                            'missedTheBuoy':'MissionFailed',
                                            'touchBuoyPreempted':'MissionPreempted'})

        # The surface task. Finishes up the mission.
        smach.StateMachine.add('NavigateToSurfacingTask', NavigateToSurfacing(), 
                                transitions={'successfulSurface':'MissionSucceeded',
                                            'unsuccessfulSurface':'MissionFailed',
                                            'surfacingPreempted':'MissionPreempted'})

    return sm

    '''
    # This threading syntax is from the old repository, and I'm not sure 
    # Where it came from originally. The threading is causing an error when trying to 
    # Call launch files, I'm removing it for now.

    # In order to get smach to respond to ctrl+c we run it in a different
    # thread and request a preempt on ctrl+c.
    smach_thread = Thread(target=lambda: sm.execute())
    smach_thread.start()

    # It is necessary to use the on_shutdown method to request the preempt
    # rather than waiting until after rospy spin to do so. Otherwise, the
    # state machine will not respond to ctrl+c.
    rospy.on_shutdown(sm.request_preempt)
    rospy.spin()
    smach_thread.join()
    '''
    
    # Execute SMACH plan
    outcome = sm.execute()

def planner_gate():
    rospy.init_node('mockMissionPlanner', anonymous=True)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['MissionFailed', 'MissionPreempted', 'MissionSucceeded'])

    # Open the container
    with sm:
        
        # Gate task. Transitions Directly to pinger task...for now.
        smach.StateMachine.add('GateTask', GateTask(), 
                                transitions={'gatePassed':'missionSucceeded',
                                            'gateMissed':'missionFailed'})

    return sm    
    # Execute SMACH plan
    # outcome = sm.execute()

if __name__ == '__main__':

    sm = planner_all_tasks()
    # sm = planner_gate()
    # ...

    #########################################
    # Preemption code
    #########################################
    
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()
    print("Tread started")
    # Wait for ctrl-c
    rospy.spin()
    print("Keyboard interrupt")
    # Request the container to preempt
    sm.request_preempt()
    print("State machine preempted")
    # Block until everything is preempted 
    # (you could do something more complicated to get the execution outcome if you want it)
    smach_thread.join()
    print("State machine joined")