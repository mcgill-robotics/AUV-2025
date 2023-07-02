import rospy
import smach
from .utility.vision import *
from .utility.functions import *
from .utility.state import *
import math

class NavigateBuoy(smach.State):
    def __init__(self, control, state, mapping, target_symbol_class, buoy_class):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state
        self.target_symbol_class = target_symbol_class
        self.buoy_class = buoy_class

    def execute(self, ud):
        print("Starting buoy navigation.") 
        origin_position = (self.state.x, self.state.y)

        gate_object = self.mapping.getClosestObject(cls=self.buoy_class, pos=(origin_position[0], origin_position[1]))
        if gate_object is None:
            print("No buoy in object map! Failed.")
            return 'failure'
    
        print("Centering and rotating in front of buoy.")
        offset_distance = -3
        offset = offset_distance * degreesToVector(gate_object[4])
        self.control.rotate((None,None,gate_object[4])) # bring to exact angle 
        self.control.move(gate_object[1] + offset[0], gate_object[2] + offset[1], gate_object[3]) # move in front of gate

        # wait and repeat just to be safe
        print("Waiting 10 seconds to improve measurement accuracy")
        rospy.sleep(10)

        print("Re-centering and rotating in front of buoy.")
        self.mapping.updateObject(gate_object)
        offset_distance = -2
        offset = offset_distance * degreesToVector(gate_object[4])
        self.control.rotate((None,None,gate_object[4])) # bring to exact angle 
        self.control.move(gate_object[1] + offset[0], gate_object[2] + offset[1], gate_object[3]) # move in front of gate

        print("Successfully centered in front of gate")

        if not self.goThrough:
            return 'success'

        self.mapping.updateObject(gate_object)
        symbol = gate_object[5] #1 if earth on left, 0 if abydos left

        if self.target_symbol == "earth": 
            if symbol >= 0.5:
                print("Going through left side")
                self.control.moveDeltaLocal((0,0.5,0))
            else : 
                print("Going through right side")
                self.control.moveDeltaLocal((0,-0.5,0))
        else: 
            if symbol <= 0.5:
                print("Going through left side")
                self.control.moveDeltaLocal((0,0.5,0))
            else: 
                print("Going through right side")
                self.control.moveDeltaLocal((0,-0.5,0))

        self.control.moveDeltaLocal((1.0 + math.sqrt((self.state.x - gate_object[1])**2 + (self.state.y - gate_object[2])**2),0,0))

        print("Successfully passed through gate!")
        
        return 'success'
