#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import Empty


# DESCRIPTION OF QUALI TASK (DETERMINED IN THE 2023 HANDBOOK):
# Setup:
#     1. Distance between starting position and gate = 3 meters
#     2. Distance between gate and pole = 10 meters
#     3. Distance between surface and top gate bar = 1 meter 
#     4. Distance between the gate's two side bars = 2 meters
# Maneuver: 
#     0. Set DVL to zero
#     1. Submerge and start 3 meters behind the gate 
#     2. Pass through the gate 
#     3. Circle around the marker 
#     4. Pass back through the gate 


class QualiQuaternion(smach.State):
    def __init__(self, control):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
    
    def execute(self, ud):
        print("Starting Quali Mission")
        # Step 0: Set DVL to zero
        pub_DVL = rospy.Publisher('/reset_state_planar', Empty, queue_size=1)
        pub_DVL.publish(Empty())

        # Step 1: Submerge 2 meters
        print("Descending 2 meters")
        self.control.move((None, None, -2.0), callback=lambda a,b: None)
        self.control.moveDelta((0,0,0), callback=lambda a,b: None)
        self.control.rotate((1, 0.0, 0.0, 0))
        rospy.sleep(2)

        # Step 2: Move 14 meters forward
        print("Moving 14 meters forward")
        self.control.moveDelta((14.0, 0.0, 0.0))
        rospy.sleep(2)

        # Step 3: Rotate 90 degrees 
        print("Rotating 90 degrees")
        self.control.rotateDelta((0.7071068, 0.0, 0.0, 0.7071068))
        rospy.sleep(2)

        # Step 4: Move 1 meter forward
        print("Moving 1 meter forward")
        self.control.moveDelta((0.0, 1.0, 0.0))
        rospy.sleep(2)

        # Step 5: Rotate 90 degrees 
        print("Rotating 90 degrees")
        self.control.rotateDelta((0.7071068, 0.0, 0.0, 0.7071068))
        rospy.sleep(2)

        # Step 6: Move 14 meters forward
        print("Moving 14 meters forward")
        self.control.moveDelta((-14.0, 0.0, 0.0))
        rospy.sleep(2)

        # Step 7: Float to surface (thrusters stop spinning - effort = 0)
        print("Stopping thrusters and floating to surface")
        self.control.kill()
        rospy.sleep(2)

        # DONE
        print("Completed")
        return 'success'  


