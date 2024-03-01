#!/usr/bin/env python3

import rospy
import smach
import numpy as np
import quaternion

# State machine for quaternion testing
class QuaternionTest(smach.StateMachine):
    def __init__(self, control):
        super().__init__(outcomes=['success', 'failure'])

        # Initial position
        initial_position = [1, 1, -1]

        # Define quaternions for rotations
        q_roll_90 = np.quaternion(0.707, 0.707, 0, 0)
        q_pitch_90 = np.quaternion(0.707, 0, 0.707, 0)
        q_yaw_90 = np.quaternion(0.707, 0, 0, 0.707)
        q_yaw_180 = np.quaternion(0, 0, 0, 1.0)
        q_arbitrary = np.quaternion(0.74222962, -0.20852589, 0.55627498, 0.31011337)

        # Add states to the state machine
        self.add('initial_position', 
                 PositionState(control=control, position=initial_position), 
                 transitions={'success': 'yaw_90', 'failure':'failure'})
        
        self.add('yaw_90', 
                 QuaternionState(control=control, quaternion=q_yaw_90), 
                 transitions={'success': 'yaw_180', 'failure':'failure'})
        
        self.add('yaw_180', 
                 QuaternionState(control=control, quaternion=q_yaw_180), 
                 transitions={'success': 'roll_90', 'failure':'failure'})
        
        self.add('roll_90', 
                 QuaternionState(control=control, quaternion=q_roll_90), 
                 transitions={'success': 'pitch_90', 'failure':'failure'})
        
        self.add('pitch_90', 
                 QuaternionState(control=control, quaternion=q_pitch_90), 
                 transitions={'success': 'arbitrary_pose', 'failure':'failure'})
        
        self.add('arbitrary_pose', 
                 QuaternionState(control=control, quaternion=q_arbitrary), 
                 transitions={'success': 'success', 'failure':'failure'})
    

# State for handling quaternion rotations
class QuaternionState(smach.State):
    def __init__(self, control, quaternion):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.quaternion = quaternion
    
    def execute(self, _):
        # Convert quaternion to list for compatibility with the control function
        quaternion_list = [self.quaternion.w, self.quaternion.x, self.quaternion.y, self.quaternion.z]
        # Call control function to rotate with the given quaternion
        self.control.rotate(quaternion_list)
        return 'success'


# State for handling position changes
class PositionState(smach.State):
    def __init__(self, control, position):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.position = position
    
    def execute(self, _):
        # Call control function to move to the specified position
        self.control.move(self.position)
        return 'success'
