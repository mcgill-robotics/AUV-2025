import rospy
import smach
from .utility.vision import *
from .utility.functions import *
import math

class NavigateGate(smach.State):
    def __init__(self, origin_class, control, state, mapping):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state
        self.origin_class = origin_class

    def execute(self, ud):
        print("Starting gate navigation.") 
        auv_current_position = self.state.getPosition()
        if self.origin_class == -1: # use auv current position as origin
            origin_position = auv_current_position
        else:
            origin_position = (0,0,0)

        gate_object = self.mapping.getClosestObject(0, (origin_position[0], origin_position[1]))

        if gate_object is None:
            print("No gate in object map! Failed.")
            return 'failure'
        
        offset_distance = -3
        offset = offset_distance * degreesToVector(gate_object[4])
        self.control.rotateYaw(gate_object[4]) # bring to exact angle 
        self.control.move(gate_object[1] + offset[0], gate_object[2] + offset[1], gate_object[3])

        print("Successfull")
        return 'success'
