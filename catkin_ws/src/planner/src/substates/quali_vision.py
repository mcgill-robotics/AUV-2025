#!/usr/bin/env python3

import rospy
import smach
from .utility.functions import *
from std_msgs.msg import Empty

class QualiVision(smach.State):
    def __init__(self, control, state, mapping, quali_gate_width):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state
        self.quali_gate_width = quali_gate_width

    def execute(self, ud):
        print("Starting quali gate navigation.") 
        #MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        self.control.move((None, None, -2), callback=lambda a,b: None)
        self.control.moveDelta((0,0,0), callback=lambda a,b: None)
        self.control.rotateEuler((0,0,None))

        gate_object = None
        while gate_object is None:
            gate_object = self.mapping.getClosestObject(cls=self.gate_class, pos=(self.state.x, self.state.y))
    
        print("Waiting 5 seconds to improve measurement accuracy")
        rospy.sleep(5)

        print("Centering and rotating in front of gate.")
        self.mapping.updateObject(gate_object)
        offset_distance = -2
        dtv = degreesToVector(gate_object[4])
        offset = [] 
        for i in range(len(dtv)):
            offset.append(offset_distance * dtv[i]) 
        self.control.rotateEuler((None,None,gate_object[4])) # bring to exact angle 
        self.control.move((gate_object[1] + offset[0], gate_object[2] + offset[1], gate_object[3])) # move in front of gate

        print("Successfully centered in front of gate")
        print("Moving to left side of gate")
        self.control.moveDeltaLocal((0.0,self.quali_gate_width/3,0.0))
        print("Moving through gate")
        self.control.moveDeltaLocal((3.0,0.0,0.0))
        print("Rotating right by 90 degrees")
        self.control.rotateDeltaEuler((0,0,-90))
        print("Moving forward")
        self.control.moveDeltaLocal((2*self.quali_gate_width/3,0.0,0.0))
        print("Rotating right by 90 degrees")
        self.control.rotateDeltaEuler((0,0,-90))
        print("Moving forward through gate")
        self.control.moveDeltaLocal((7.0,0.0,0.0))

        print("Finished quali!")
        
        return 'success'
