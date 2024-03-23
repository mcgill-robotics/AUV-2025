import rospy
import smach
from .utility.functions import *

class NavigateGate(smach.State):
    def __init__(self, control, state, mapping, goThrough, gate_width):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state
        self.goThrough = goThrough
        self.gate_width = gate_width

    def execute(self, ud):
        print("Starting gate navigation.") 
        #MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        self.control.move((None, None, -1))
        self.control.rotateEuler((0,0,None))

        gate_object = self.mapping.getClosestObject(cls="Gate", pos=(self.state.x, self.state.y))
        if gate_object is None:
            print("No gate in object map! Failed.")
            return 'failure'
    
        print("Centering and rotating in front of gate.")
        offset_distance = -3
        gate_rot = 180 if gate_object[4] is None else gate_object[4]
        dtv = degreesToVector(gate_rot)
        offset = [] 
        for i in range(len(dtv)):
            offset.append(offset_distance * dtv[i]) 
        self.control.rotateEuler((None,None,gate_rot)) # bring to exact angle 
        self.control.move((gate_object[1] + offset[0], gate_object[2] + offset[1], gate_object[3])) # move in front of gate

        # wait and repeat just to be safe
        print("Waiting 10 seconds to improve measurement accuracy")
        rospy.sleep(10)

        print("Re-centering and rotating in front of gate.")
        self.mapping.updateObject(gate_object)
        gate_rot = 180 if gate_object[4] is None else gate_object[4]
        dtv = degreesToVector(gate_rot)
        offset = [] 
        for i in range(len(dtv)):
            offset.append(offset_distance * dtv[i]) 
        self.control.rotateEuler((None,None,gate_rot)) # bring to exact angle 
        self.control.move((gate_object[1] + offset[0], gate_object[2] + offset[1], gate_object[3])) # move in front of gate

        print("Successfully centered in front of gate")

        # The side of the gate that red corresponds to (left ro right)
        red_gate_side = rospy.get_param("red_side")

        # The target colour to follow (blue or red)
        target_colour = rospy.get_param("target_colour")

        print("Red is on the {} side. Target colour is {}".format(red_gate_side, target_colour))

        if not self.goThrough:
            return 'success'

        self.mapping.updateObject(gate_object)

        if target_colour == "red": 
            if red_gate_side == "left":
                print("Targeting RED side (left). Going through left side")
                self.control.moveDeltaLocal((0,self.gate_width/4,0)) # a quarter of gate width
            else: 
                print("Targeting RED side (right). Going through right side")
                self.control.moveDeltaLocal((0,-self.gate_width/4,0)) # a quarter of gate width
        else: 
            if red_gate_side == "right":
                print("Targeting BLUE side (left). Going through left side")
                self.control.moveDeltaLocal((0,self.gate_width/4,0)) # a quarter of gate width
            else: 
                print("Targeting BLUE side (right). Going through right side")
                self.control.moveDeltaLocal((0,-self.gate_width/4,0)) # a quarter of gate width

        self.control.moveDeltaLocal((5.0,0.0,0.0))

        print("Successfully passed through gate!")
        
        return 'success'
