import rospy
import actionlib
from auv_msgs.msg import QuaternionRotationAction
from geometry_msgs.msg import Quaternion, Pose
import numpy as np

class QuaternionServer():
    def __init__(self):
        self.server = actionlib.SimpleActionServer('quaternion_rotation', QuaternionRotationAction, execute_cb= self.callback, auto_start = False)
        self.orientation = np.quaternion([0]*4)

        self.sub = rospy.Subcriber("pose",Pose,self.set)

        self.intertial_tensor = np.eye(3)
    
    def set(self,data):
        curr = data.orientation
        self.orientation = np.quaternion(curr)

    def callback(self,goal):
        goalquat = np.quaternion(goal.rotation)
        deltaQuat = goalquat * np.quaternion.conjugate(self.orientation)
        position_axis = np.quaternion.as_rotation_vector(deltaQuat)

        norm = np.norm(position_axis)
        norm_position_axis = position_axis / norm

        torque_axis = np.matmul(self.intertial_tensor,norm_position_axis)



