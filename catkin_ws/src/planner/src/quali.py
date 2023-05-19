import rospy
import smach

from substates.breadth_first_search import *
from substates.grid_search import *
from substates.linear_search import *
from substates.navigate_lane_marker import *
from substates.test_submerged_rotations import *
from substates.utility.controller import Controller


def endMission(msg="Shutting down mission planner."):
    print(msg)
    control.preemptCurrentAction()
    control.velocity((0,0,0))
    control.angularVelocity((0,0,0))

def qualiMission():
    # Step 1: Submerge 2 meters
    print("Descending 2 meters")
    control.moveDelta((0, 0, -2.0))
    rospy.sleep(2)

    # Step 2: Move 14 meters forward
    print("Moving 14 meters forward")
    control.moveDeltaLocal((14.0, 0, 0))
    rospy.sleep(2)

    # Step 3: Rotate 90 degrees 
    print("Rotating 90 degrees")
    control.rotateDelta((0, 0, 90))
    rospy.sleep(2)

    # Step 4: Move 1 meter forward
    print("Moving 1 meter forward")
    control.moveDeltaLocal((1.0, 0, 0))
    rospy.sleep(2)

    # Step 5: Rotate 90 degrees 
    print("Rotating 90 degrees")
    control.rotateDelta((0, 0, 90))
    rospy.sleep(2)

    # Step 6: Move 14 meters forward
    print("Moving 14 meters forward")
    control.moveDeltaLocal((14.0, 0, 0))
    rospy.sleep(2)

    # Step 7: Float to surfice (thrusters stop spinning - effort = 0)
    print("Stopping thrusters and floating to surface")
    control.velocity((0, 0, 0))
    rospy.sleep(2)

    # DONE
    print("Completed")



if __name__ == '__main__':
    rospy.init_node('mission_planner',log_level=rospy.DEBUG)
    rospy.on_shutdown(endMission)

    control = Controller()

    print("Mission is over.")
    # DESCRIPTION OF QUALI TASK (DETERMINED IN THE 2023 HANDBOOK):
    # Setup:
    #     1. Distance between starting position and gate = 3 meters
    #     2. Distance between gate and pole = 10 meters
    #     3. Distance between surface and top gate bar = 1 meter 
    #     4. Distance between the gate's two side bars = 2 meters
    # Maneuver: 
    #     1. Submerge and start 3 meters behind the gate 
    #     2. Pass through the gate 
    #     3. Circle around the marker 
    #     4. Pass back through the gate 
    