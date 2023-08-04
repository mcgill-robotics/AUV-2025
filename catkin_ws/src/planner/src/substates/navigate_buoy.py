import rospy
import smach
from .utility.functions import *

class NavigateBuoy(smach.State):
    def __init__(self, control, state, mapping, target_symbol, buoy_width, buoy_height):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state
        self.target_symbol = target_symbol
        self.buoy_width = buoy_width
        self.buoy_height = buoy_height

    def execute(self, ud):
        print("Starting buoy navigation.") 
        #MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        self.control.move((None, None, -2))
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

        buoy_symbol_locations = 12 if buoy_object[5] is None else buoy_object[5]
        first_buoy_symbol_location = math.floor(buoy_symbol_locations / 10)
        second_buoy_symbol_location = buoy_symbol_locations - 10 * math.floor(buoy_symbol_locations / 10)

        for symbol_location in [first_buoy_symbol_location, second_buoy_symbol_location]:
            self.control.move(homing_position) # move in front of buoy
            if symbol_location == 1: # TOP LEFT
                self.control.moveDeltaLocal(offset_distance, self.buoy_width/4, self.buoy_height/4)
            elif symbol_location == 2: # TOP RIGHT
                self.control.moveDeltaLocal(offset_distance, -self.buoy_width/4, self.buoy_height/4)
            elif symbol_location == 3: # BOTTOM LEFT
                self.control.moveDeltaLocal(offset_distance, self.buoy_width/4, -self.buoy_height/4)
            elif symbol_location == 4: # BOTTOM RIGHT
                self.control.moveDeltaLocal(offset_distance, -self.buoy_width/4, -self.buoy_height/4)


        print("Successfully completed buoy task!")
        
        return 'success'
