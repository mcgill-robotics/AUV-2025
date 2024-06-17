import rospy
import smach
from .utility.functions import *

class NavigateGate(smach.State):
    def __init__(self, control, mapping, state, goThrough):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state
        if target not in ["red", "blue"]:
            raise ValueError("Target must be red or blue.")
        self.goThrough = goThrough
        self.target_colour = rospy.get_param("target_colour")
        self.gate_width = rospy.get_param("gate_width")
        self.red_gate_side = rospy.get_param("red_side")

    def is_preempted(self):
        if self.preempt_requested():
            print("IPS being preempted")
            self.service_preempt()
            return 'failure'


    def execute(self, ud):
        print("Starting gate navigation.") 
        #MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        self.control.move((None, None, -1))
        self.control.flatten()

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
        self.is_preempted()
        self.control.rotateEuler((None,None,gate_rot)) # bring to exact angle 
        self.control.move((gate_object[1] + offset[0], gate_object[2] + offset[1], gate_object[3])) # move in front of gate

        # wait and repeat just to be safe
        print("Waiting to improve measurement accuracy")
        rospy.sleep(rospy.get_param("object_observation_time"))

        print("Re-centering and rotating in front of gate.")
        self.mapping.updateObject(gate_object)
        gate_rot = 180 if gate_object[4] is None else gate_object[4]
        dtv = degreesToVector(gate_rot)
        offset = [] 
        for i in range(len(dtv)):
            offset.append(offset_distance * dtv[i]) 
        self.is_preempted()
        self.control.rotateEuler((None,None,gate_rot)) # bring to exact angle 
        self.control.move((gate_object[1] + offset[0], gate_object[2] + offset[1], gate_object[3])) # move in front of gate

        print("Successfully centered in front of gate")

        if not self.goThrough:
            return 'success'

        print("Red is on the {} side. Target colour is {}".format(self.red_gate_side, self.target_colour))

        self.mapping.updateObject(gate_object)
        symbol = 0 if gate_object[5] is None else gate_object[5] #1 if earth on left, 0 if abydos left

        self.is_preempted()

        if self.target == "red": 
            if red_gate_side == "left":
                print("Going through left side")
                self.control.moveDeltaLocal((0,self.gate_width/4,0)) # a quarter of gate width
            else: 
                print("Going through right side")
                self.control.moveDeltaLocal((0,-self.gate_width/4,0)) # a quarter of gate width
        else: 
            if red_gate_side == "right":
                print("Going through left side")
                self.control.moveDeltaLocal((0,self.gate_width/4,0)) # a quarter of gate width
            else: 
                print("Going through right side")
                self.control.moveDeltaLocal((0,-self.gate_width/4,0)) # a quarter of gate width

        self.is_preempted()
        self.control.moveDeltaLocal((5.0,0.0,0.0))

        print("Successfully passed through gate!")
        
        return 'success'