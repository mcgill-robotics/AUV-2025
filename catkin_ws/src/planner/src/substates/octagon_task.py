#!/usr/bin/env python3
import smach
import rospy

class NavigateOctagon(smach.State):
    def __init__(self, control, mapping, state):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state

    def execute(self, ud):
        print("Starting octagon navigation.") 
        #MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        self.control.move((None, None, rospy.get_param("down_cam_search_depth")))
        self.control.flatten()
        if self.preempt_requested():
                print("IPS being preempted")
                self.service_preempt()
                return 'failure'

        auv_current_position = (self.state.x, self.state.y)
       
        octagon_obj = self.mapping.getClosestObject("Octagon Table", (auv_current_position[0], auv_current_position[1]))
        
        if octagon_obj is None:
            print("No octagon in object map! Failed.")
            return 'failure'
        
        print("Moving to the center of the octagon.")
        self.control.move((octagon_obj[1], octagon_obj[2], rospy.get_param("down_cam_search_depth")), face_destination=True)
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
        self.control.move((None, None, rospy.get_param("down_cam_search_depth")))
        if self.preempt_requested():
                print("IPS being preempted")
                self.service_preempt()
                return 'failure'
        self.control.move((self.search_point[0], self.search_point[1], None), face_destination=True)
        return 'success'