#!/usr/bin/env python3
import smach

class NavigateOctagon(smach.State):
    def __init__(self, control, state, mapping):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state

    def execute(self, ud):
        print("Starting octagon navigation.") 
        #MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        self.control.move((None, None, -2))
        self.control.rotateEuler((0,0,None))

        auv_current_position = (self.state.x, self.state.y)
       
        octagon_obj = self.mapping.getClosestObject("Octagon Table", (auv_current_position[0], auv_current_position[1]))
        
        if octagon_obj is None:
            print("No octagon in object map! Failed.")
            return 'failure'
        
        print("Moving to the center of the octagon.")
        self.control.move((octagon_obj[1], octagon_obj[2], -2))
        print("Surfacing.")
        self.control.kill()

        print("Successfully navigated the octagon.")
        return 'success'

class GoToOctagon(smach.State):
    def __init__(self,search_point, control):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.search_point = search_point

    def execute(self, ud):
        print("Moving up to avoid the buoy.")
        self.control.move((None, None, -1))
        self.control.move((self.search_point[0], self.search_point[1], None), face_destination=True)
        return 'success'