#!/usr/bin/env python3

import smach

class NavigateOctagon(smach.State):
    def __init__(self, control, state, mapping):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state

    def execute(self, ud):
        # Start octagon navigation
        print("Starting octagon navigation.") 

        # Move to middle of pool depth and flat orientation
        self.control.move((None, None, -2))
        self.control.rotateEuler((0, 0, None))

        # Get current AUV position
        auv_current_position = (self.state.x, self.state.y)
       
        # Find the closest octagon object
        octagon_obj = self.mapping.getClosestObject("Octagon Table", auv_current_position)
        
        # Check if octagon object is found
        if octagon_obj is None:
            print("No octagon in object map! Failed.")
            return 'failure'
        
        # Move to the center of the octagon
        print("Moving to the center of the octagon.")
        self.control.move((octagon_obj[1], octagon_obj[2], -2), face_destination=True)
        
        # Moving to the surface
        print("Surfacing.")
        self.control.kill()

        # Successfully navigated the octagon
        print("Successfully navigated the octagon.")
        return 'success'

class GoToOctagon(smach.State):
    def __init__(self, search_point, control):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.search_point = search_point

    def execute(self, userdata):
        # Move up to avoid the buoy
        print("Moving up to avoid the buoy.")
        self.control.move((None, None, -1))
        self.control.move((self.search_point[0], self.search_point[1], None), face_destination=True)
        return 'success'
