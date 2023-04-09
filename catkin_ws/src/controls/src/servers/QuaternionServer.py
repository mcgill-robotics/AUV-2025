import rospy
import actionlib
from auv_msgs.msg import QuaternionRotationAction
from geometry_msgs.msg import Quaternion, Pose
import numpy as np
import quaternion

class QuaternionServer():
    def __init__(self):
        self.server = actionlib.SimpleActionServer('quaternion_rotation', QuaternionRotationAction, execute_cb= self.callback, auto_start = False)
        self.orientation = np.quaternion([0]*4)

        self.sub = rospy.Subcriber("pose",Pose,self.set)
    
    def set(self,data):
        curr = data.orientation
        self.orientation = np.quaternion(curr)

    def callback(self,goal):
        goalquat = np.quaternion(goal.rotation)
        deltaQuat = goalquat * np.quaternion.conjugate(self.orientation)
        axis = np.quaternion.as_rotation_vector(deltaQuat)

        x,y,z = axis

        
