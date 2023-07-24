#!/usr/bin/env python3

import rospy
import smach
import numpy as np
import quaternion

class QuaternionTest(smach.StateMachine):
    def __init__(self, control):
        super().__init__(outcomes=['success', 'failure'])

        init_pos = [1, 1, -1]
        q_roll_90 = np.quaternion(0.707, 0.707, 0, 0)
        q_pitch_90 = np.quaternion(0.707, 0, 0.707, 0)
        q_yaw_90 = np.quaternion(0.707, 0, 0, 0.707)
        q_yaw_180 = np.quaternion(0, 0, 0, 1.0)
        q_arbitrary = np.quaternion(0.74222962, -0.20852589, 0.55627498, 0.31011337)

        self.add('initial_position', PositionState(control=control, position=init_pos), 
                transitions={'success': 'yaw_90', 'failure':'failure'})
        self.add('yaw_90', QuaternionState(control=control, quaternion=q_yaw_90), 
                transitions={'success': 'yaw_180', 'failure':'failure'})
        self.add('yaw_180', QuaternionState(control=control, quaternion=q_yaw_180), 
                transitions={'success': 'roll_90', 'failure':'failure'})
        self.add('roll_90', QuaternionState(control=control, quaternion=q_roll_90), 
                transitions={'success': 'pitch_90', 'failure':'failure'})
        self.add('pitch_90', QuaternionState(control=control, quaternion=q_pitch_90), 
                transitions={'success': 'aribitrary_pose', 'failure':'failure'})
        self.add('arbitrary_pose', QuaternionState(control=control, quaternion=q_arbitrary), 
                transitions={'success': 'success', 'failure':'failure'})
    

class QuaternionState(smach.State):
    def __init__(self, control, quaternion):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.q = quaternion
    
    def execute(self, _):
        quaternion = [self.q.w, self.q.x, self.q.y, self.q.z]
        self.control.rotate(quaternion)
        return 'success'


class PositionState(smach.State):
    def __init__(self, control, position):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.pos = position
    
    def execute(self, _):
        self.control.move(self.pos)
        return 'success'
