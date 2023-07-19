import rospy
import smach
from .utility.functions import *

class NavigateBuoy(smach.State):
    def __init__(self, control, state, mapping, target_symbol):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state
        self.target_symbol = target_symbol

    def execute(self, ud):
        print("Starting buoy navigation.") 
        #MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        self.control.move((None, None, -2), callback=lambda a,b: None)
        self.control.moveDelta((0,0,0), callback=lambda a,b: None)
        self.control.rotateEuler((0,0,None))

        buoy_object = self.mapping.getClosestObject(cls="Buoy", pos=(self.state.x, self.state.y))
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
        self.control.move((buoy_object[1] + offset[0], buoy_object[2] + offset[1], buoy_object[3])) # move in front of buoy

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
        self.control.move(homing_position) # move in front of buoy
        print("Successfully centered in front of gate")

        buoy_symbols = self.mapping.getClass(cls=self.target_symbol)
        if len(buoy_symbols) == 0:
            print("No buoy symbol in object map! Failed.")
            return 'failure'

        for symbol in buoy_symbols:
            print("Touching symbol.")
            self.control.move(symbol[1:4]) # move to symbol
            print("Returning to homing position.")
            self.control.move(homing_position) # move in front of buoy

        print("Successfully completed buoy task!")
        
        return 'success'
