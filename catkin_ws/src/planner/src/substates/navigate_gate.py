import rospy
import smach
from .utility.functions import degreesToVector

class NavigateGate(smach.State):
    def __init__(self, control, state, mapping, target_symbol, go_through, gate_width):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state
        if target_symbol not in ["Earth Symbol", "Abydos Symbol"]:
            raise ValueError("Target symbol must be either 'Earth Symbol' or 'Abydos Symbol'.")
        self.target_symbol = target_symbol
        self.go_through = go_through
        self.gate_width = gate_width

    def execute(self, ud):
        print("Starting gate navigation.") 
        
        # Move to the middle of the pool depth and flat orientation
        self.control.move((None, None, -1))
        self.control.rotateEuler((0, 0, None))

        # Find the closest gate object
        gate_object = self.mapping.getClosestObject(cls="Gate", pos=(self.state.x, self.state.y))
        if gate_object is None:
            print("No gate in object map! Navigation failed.")
            return 'failure'
    
        print("Centering and rotating in front of the gate.")
        offset_distance = -3
        gate_rot = 180 if gate_object[4] is None else gate_object[4]
        dtv = degreesToVector(gate_rot)
        offset = [offset_distance * dtv[i] for i in range(len(dtv))]
        self.control.rotateEuler((None, None, gate_rot))  # Bring to exact angle 
        self.control.move((gate_object[1] + offset[0], gate_object[2] + offset[1], gate_object[3]))  # Move in front of gate

        # Wait and repeat just to be safe
        print("Waiting 10 seconds to improve measurement accuracy")
        rospy.sleep(10)

        print("Re-centering and rotating in front of the gate.")
        self.mapping.updateObject(gate_object)
        gate_rot = 180 if gate_object[4] is None else gate_object[4]
        dtv = degreesToVector(gate_rot)
        offset = [offset_distance * dtv[i] for i in range(len(dtv))]
        self.control.rotateEuler((None, None, gate_rot))  # Bring to exact angle 
        self.control.move((gate_object[1] + offset[0], gate_object[2] + offset[1], gate_object[3]))  # Move in front of gate

        print("Successfully centered in front of the gate.")

        if not self.go_through:
            return 'success'

        self.mapping.updateObject(gate_object)
        symbol = 0 if gate_object[5] is None else gate_object[5]  # 1 if earth on the left, 0 if abydos on the left

        if self.target_symbol == "Earth Symbol": 
            if symbol >= 0.5:
                print("Going through the left side of the gate.")
                self.control.moveDeltaLocal((0, self.gate_width/4, 0))  # Move a quarter of gate width
            else: 
                print("Going through the right side of the gate.")
                self.control.moveDeltaLocal((0, -self.gate_width/4, 0))  # Move a quarter of gate width
        else: 
            if symbol <= 0.5:
                print("Going through the left side of the gate.")
                self.control.moveDeltaLocal((0, self.gate_width/4, 0))  # Move a quarter of gate width
            else: 
                print("Going through the right side of the gate.")
                self.control.moveDeltaLocal((0, -self.gate_width/4, 0))  # Move a quarter of gate width

        # Move a certain distance after passing through the gate
        self.control.moveDeltaLocal((5.0, 0.0, 0.0))

        print("Successfully passed through the gate!")
        
        return 'success'
