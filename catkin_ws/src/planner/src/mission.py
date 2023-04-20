#!/usr/bin/env python3

import rospy
import smach

from substates.breadth_first_search import *
from substates.grid_search import *
from substates.linear_search import *
from substates.navigate_lane_marker import *
from substates.test_submerged_rotations import *
from substates.utility.controller import Controller

def descend(depth):
    descended = False
    def done():
        global descended
        descended = True
    control.moveDelta((0, 0, depth), done)
    while not descended: rospy.sleep(0.1)

def endMission(msg="Shutting down mission planner."):
    print(msg)
    control.preemptCurrentAction()
    control.velocity((0,0,0))
    control.angularVelocity((0,0,0))

def testRotationsMission():
    descend(depth=-2.0)
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('test_submerged_rotations', TestSubmergedRotations(hold_time = 5.0, control=control), 
                transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished rotation test mission.")

def laneMarkerGridSearchMission():
    descend(depth=-0.5)
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('gridsearch', GridSearch(timeout=60, target_classes=[0], control=control), 
                transitions={'success': 'navigateLaneMarker', 'failure':'failure'})
        smach.StateMachine.add('navigateLaneMarker', NavigateLaneMarker(control=control), 
                transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished lane marker grid search mission. Result: {}".format(res))

def testControllerMission(testsToPerform=[0,1,2,3,4,5,6,7,8]):
    def done():
        pass
        #print("Done.")
    if 0 in testsToPerform:
        #submerge and cancel ascension: AUV should stay underwater when the action was preempted
        print("Descending 2 meters.")
        control.moveDelta((0, 0, -2.0))
        print("Ascending 2 meters.")
        control.moveDelta((0, 0, 2.0), done)
        print("Cancelling.")
        control.preemptCurrentAction()

    if 1 in testsToPerform:
        #move by one meter and back
        print("Moving to origin.")
        control.move((0,0,0))
        print("Moving AUV to (0,0,-1.0).")
        control.move((0,0,-1.0))
        print("Moving to origin.")
        control.move((0,0,0))

    if 2 in testsToPerform:
        #rotate 90 degrees and return to original rotation
        print("Setting orientation to (0,0,0)")
        control.rotate((0,0,0))
        print("Setting orientation to (0,0,90)")
        control.rotate((0,0,90))
        print("Setting orientation to (0,0,0)")
        control.rotate((0,0,0))

    if 3 in testsToPerform:
        #go in a square
        for i in range(4):
            print("Surging by one meter.")
            control.moveDeltaLocal((1,0,0))
            print("Adding (0,0,90) orientation")
            control.rotateDelta((0,0,90))

    if 4 in testsToPerform:
        #two move deltas in quick succession: should move a total of 2 meters, then return to original location
        print("Surging by one meter.")
        control.moveDeltaLocal((1,0,0), done)
        print("Surging by one meter again.")
        control.moveDeltaLocal((1,0,0))
        print("Surging backwards by two meters.")
        control.moveDeltaLocal((-2,0,0))

    if 5 in testsToPerform:
        #should yaw at a constant rate then stop after two seconds
        print("Applying constant angular velocity of (0,0,10).")
        control.angularVelocity((0,0,10))
        rospy.sleep(2)
        print("Setting angular velocity to (0,0,0).")
        control.angularVelocity((0,0,0))

    if 6 in testsToPerform:
        #should start off spinning slow and over the span of 5 seconds increase angular speed
        for i in range(5):
            print("Applying angular velocity of (0,0,2).")
            control.deltaAngularVelocity((0,0,2))
            rospy.sleep(1)
        print("Setting angular velocity to (0,0,0).")
        control.angularVelocity((0,0,0))

    if 7 in testsToPerform:
        #should move on x at a constant velocity then stop all thruster movement after 2 seconds
        print("Applying constant velocity of (1,0,0).")
        control.velocity((1,0,0))
        rospy.sleep(2)
        print("Setting thruster velocity to (0,0,0).")
        control.velocity((0,0,0))
        rospy.sleep(2)

    if 8 in testsToPerform:
        #should surge at a constant velocity then stop all thruster movement after 2 seconds
        print("Applying constant surge of 1.")
        control.velocityLocal((1,0,0))
        rospy.sleep(2)
        print("Setting local velocity to (0,0,0).")
        control.velocityLocal((0,0,0))
        rospy.sleep(2)
    
    if 9 in testsToPerform:
        #should submerge, move on x at a constant velocity, then stop where it is (stay submerged), then float to surface (thrusters should stop spinning)
        print("Descending 1 meters.")
        control.moveDelta((0, 0, -1.0))
        print("Adding delta world velocity of (1,0,0).")
        control.deltaVelocity((1,0,0))
        rospy.sleep(2)
        print("Adding delta world velocity of (-1,0,0).")
        control.deltaVelocity((-1,0,0))
        rospy.sleep(2)
        print("Setting local velocity to (0,0,0).")
        control.velocity((0,0,0))
    
    if 10 in testsToPerform:
        #should submerge, surge at a constant velocity, then stop where it is (stay submerged), then float to surface (thrusters should stop spinning)
        print("Descending 1 meters.")
        control.moveDelta((0, 0, -1.0))
        print("Adding delta local velocity of (1,0,0).")
        control.deltaVelocityLocal((1,0,0))
        rospy.sleep(2)
        print("Adding delta local velocity of (-1,0,0).")
        control.deltaVelocityLocal((-1,0,0))
        rospy.sleep(2)
        print("Setting local velocity to (0,0,0).")
        control.velocityLocal((0,0,0))
    
    #return to initial position
    print("Moving to origin.", done)
    control.move((0,0,0))
    print("Setting orientation to (0,0,0)")
    control.rotate((0,0,0))

    endMission("Controller test passed.")


if __name__ == '__main__':
    rospy.init_node('mission_planner')
    rospy.on_shutdown(endMission)

    control = Controller()

    # ----- UNCOMMENT BELOW TO RUN MISSION(S) -----
    
    testControllerMission(testsToPerform=[0,1,2,3,4,5,6,7,8])
        # CONTROLLER TEST DESCRIPTIONS:
        # 0: submerge and cancel ascension: AUV should stay underwater when the action was preempted
        # 1: move by one meter and back
        # 2: rotate 90 degrees and return to original rotation
        # 3: go in a square
        # 4: two move deltas in quick succession: should move a total of 2 meters, then return to original location
        # 5: should yaw at a constant rate then stop after two seconds
        # 6: should start off spinning slow and over the span of 5 seconds increase angular speed every second
        # 7: should move on x at a constant velocity then stop all thruster movement after 2 seconds
        # 8: should surge at a constant velocity then stop all thruster movement after 2 seconds
        # 9: should submerge, move on x at a constant velocity, then stop where it is (stay submerged), then float to surface (thrusters should stop spinning)
        # 10: should submerge, surge at a constant velocity, then stop where it is (stay submerged), then float to surface (thrusters should stop spinning)
        # AFTER ALL: return to 0,0,0 and set orientation to 0,0,0 at the same time
    #testRotationsMission()
    #laneMarkerGridSearchMission()
