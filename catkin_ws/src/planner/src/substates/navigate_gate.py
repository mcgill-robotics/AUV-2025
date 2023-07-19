import rospy
import smach
from .utility.functions import *

class NavigateGate(smach.State):
    def __init__(self, control, state, mapping, target_symbol, goThrough, gate_width):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state
        if target_symbol not in ["Earth Symbol", "Abydos Symbol"]:
            raise ValueError("Target symbol must be one of Earth Symbol or Abydos Symbol.")
        self.target_symbol = target_symbol
        self.goThrough = goThrough
        self.gate_width = gate_width

    def execute(self, ud):
        print("Starting gate navigation.") 
        #MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        self.control.move((None, None, -2))
        self.control.rotateEuler((0,0,None))

        gate_object = self.mapping.getClosestObject(cls="Gate", pos=(self.state.x, self.state.y))
        if gate_object is None:
            print("No gate in object map! Failed.")
            return 'failure'
    
        print("Centering and rotating in front of gate.")
        offset_distance = -3
        dtv = degreesToVector(gate_object[4])
        offset = [] 
        for i in range(len(dtv)):
            offset.append(offset_distance * dtv[i]) 
        self.control.rotateEuler((None,None,gate_object[4])) # bring to exact angle 
        self.control.move((gate_object[1] + offset[0], gate_object[2] + offset[1], gate_object[3])) # move in front of gate

        # wait and repeat just to be safe
        print("Waiting 10 seconds to improve measurement accuracy")
        rospy.sleep(10)

        print("Re-centering and rotating in front of gate.")
        self.mapping.updateObject(gate_object)
        dtv = degreesToVector(gate_object[4])
        offset = [] 
        for i in range(len(dtv)):
            offset.append(offset_distance * dtv[i]) 
        self.control.rotateEuler((None,None,gate_object[4])) # bring to exact angle 
        self.control.move((gate_object[1] + offset[0], gate_object[2] + offset[1], gate_object[3])) # move in front of gate

        print("Successfully centered in front of gate")

        if not self.goThrough:
            return 'success'

        self.mapping.updateObject(gate_object)
        symbol = gate_object[5] #1 if earth on left, 0 if abydos left

        if self.target_symbol == "Earth Symbol": 
            if symbol >= 0.5:
                print("Going through left side")
                self.control.moveDeltaLocal((0,self.gate_width/4,0)) # a quarter of gate width
            else : 
                print("Going through right side")
                self.control.moveDeltaLocal((0,-self.gate_width/4,0)) # a quarter of gate width
        else: 
            if symbol <= 0.5:
                print("Going through left side")
                self.control.moveDeltaLocal((0,self.gate_width/4,0)) # a quarter of gate width
            else: 
                print("Going through right side")
                self.control.moveDeltaLocal((0,-self.gate_width/4,0)) # a quarter of gate width

        self.control.moveDeltaLocal((5.0,0.0,0.0))

        print("Successfully passed through gate!")
        
        return 'success'
