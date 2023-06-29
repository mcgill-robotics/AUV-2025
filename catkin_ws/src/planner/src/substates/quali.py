#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import Float64


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


class Quali(smach.State):
    def __init__(self, control=None):
        super().__init__(outcomes=['success', 'failure'])
        if control == None: raise ValueError("control argument is None")
        self.control = control
    
    def execute(self, ud):
        print("Starting Quali Mission")
        try:
            # Step 0: Set DVL to zero
            # pub_DVL = rospy.Publisher('/[DVL_something]', Float64, queue_size=1)
            # pub_DVL.publish((0, 0, 0))

            # Step 1: Submerge 2 meters
            print("Descending 2 meters")
            self.control.move((0, 0, -2.0))
            self.control.rotate((None,None,0))
            rospy.sleep(2)

            # Step 2: Move 14 meters forward
            print("Moving 14 meters forward")
            self.control.move((14.0, 0, -2.0))
            rospy.sleep(2)

            # Step 3: Rotate 90 degrees 
            print("Rotating 90 degrees")
            self.control.rotate((None,None,-90))
            rospy.sleep(2)

            # Step 4: Move 1 meter forward
            print("Moving 1 meter forward")
            self.control.move((14.0, -1, -2.0))
            rospy.sleep(2)

            # Step 5: Rotate 90 degrees 
            print("Rotating 90 degrees")
            self.control.rotate((None,None,-180))
            rospy.sleep(2)

            # Step 6: Move 14 meters forward
            print("Moving 14 meters forward")
            self.control.move((0.0, -1, -2.0))
            rospy.sleep(2)

            # Step 7: Float to surface (thrusters stop spinning - effort = 0)
            print("Stopping thrusters and floating to surface")
            self.control.kill()
            rospy.sleep(2)

            # DONE
            print("Completed")
            return 'success'

        except KeyboardInterrupt:
            print("Quali mission interrupted by used.")
            return 'failure'    

    
