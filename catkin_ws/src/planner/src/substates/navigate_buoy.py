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

        buoy_object = self.mapping.getClosestObject(cls=self.buoy_class, pos=(self.state.x, self.state.y))
        if buoy_object is None:
            print("No buoy in object map! Failed.")
            return 'failure'
    
        print("Centering and rotating in front of buoy.")
        offset_distance = -2
        dtv = degreesToVector(buoy_object[4])
        offset = [] 
        for i in range(len(dtv)):
            offset.append(offset_distance * dtv[i]) 
        self.control.rotateEuler((None,None,buoy_object[4])) # bring to exact angle 
        self.control.move((buoy_object[1] + offset[0], buoy_object[2] + offset[1], buoy_object[3])) # move in front of gate

        # wait and keep measuring just to be safe
        print("Waiting 10 seconds to improve measurement accuracy")
        rospy.sleep(10)

        print("Re-centering and rotating in front of buoy.")
        self.mapping.updateObject(buoy_object)
        dtv = degreesToVector(buoy_object[4])
        offset = [] 
        for i in range(len(dtv)):
            offset.append(offset_distance * dtv[i]) 
        homing_rotation = (None,None,buoy_object[4])
        homing_position = (buoy_object[1] + offset[0], buoy_object[2] + offset[1], buoy_object[3])

        self.control.rotateEuler(homing_rotation) # bring to exact angle 
        self.control.move(homing_position) # move in front of gate
        print("Successfully centered in front of gate")

        buoy_symbols = self.mapping.getClass(cls=self.target_symbol_class)
        if len(buoy_symbols) == 0:
            print("No buoy symbol in object map! Failed.")
            return 'failure'

        for symbol in buoy_symbols:
            print("Touching symbol.")
            self.control.move(symbol[1:4]) # move in front of gate
            print("Returning to homing position.")
            self.control.move(homing_position) # move in front of gate

        print("Successfully completed buoy task!")
        
        return 'success'
