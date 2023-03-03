#! /usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose
from auv_msgs.msg import StateAction, StateFeedback, StateResult
from std_msgs.msg import Float64



import math
 


class StateControlActionServer():

    def __init__(self) -> None:
        self.server = actionlib.SimpleActionServer('state_control_action', StateAction, execute_cb= self.callback, auto_start = False)
        self.feedback = StateFeedback()
        self.result = StateResult()

        #self.pub_x = rospy.Publisher('', Float64, queue_size=50)
        #self.pub_y = rospy.Publisher('', Float64, queue_size=50)
        self.pub_z = rospy.Publisher('z_setpoint', Float64, queue_size=50)
        self.pub_theta_x = rospy.Publisher('theta_x_setpoint', Float64, queue_size=50)
        self.pub_theta_y = rospy.Publisher('theta_y_setpoint', Float64, queue_size=50)
        self.pub_theta_z = rospy.Publisher('theta_z_setpoint', Float64, queue_size=50)
        self.depth = 0
        
        self.server.start()

        self.position = None
        self.roll = None
        self.pitch = None
        self.yaw = None

        self.sub = rospy.Subscriber("pose",Pose,self.set)
    
    def set(self,data):
        self.position = data.position
        self.roll, self.pitch, self.yaw = euler_from_quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)

    #stolen from the internet
    def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    # receives a new goal 
    # def callback(self, goal):

    #     # set the PIDs
    #     self.pub.publish(goal.setpoint)

    #     # monitor when reached pose
    #     while(self.depth < goal.setpoint - 0.5 or self.depth > goal.setpoint + 0.5):
    #         continue

    #     self.result = self.feedback
    #     rospy.loginfo("Succeeded")
    #     self.server.set_succeeded(self.result)

    def callback(self, goal):

        # set the PIDs
        pose = goal.pose


        self.publish_setpoints(pose)

        # monitor when reached pose
        while(not self.check_status(pose)):
            rospy.sleep(0.1)

        self.result = self.feedback
        rospy.loginfo("Succeeded")
        self.server.set_succeeded(self.result)

    def check_status(self,pose):
        if(self.position == None or self.roll == None or self.pitch == None or self.yaw == None):
            return False

        roll, pitch, yaw = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        tolerance_position = 0.5
        tolerance_orientation = 1

        #x_diff = abs(self.position.x - pose.position.x) <= tolerance_position
        #y_diff = abs(self.position.y - pose.position.x) <= tolerance_position
        z_diff = abs(self.position.z - pose.position.x) <= tolerance_orientation
        theta_x_diff = abs(self.roll - roll) <= tolerance_roation
        theta_y_diff = abs(self.pitch - pitch) <= tolerance_roation
        theta_z_diff = abs(self.yaw - yaw) <= tolerance_roation

        return x_diff and y_diff and z_diff and theta_x_diff and theta_y_diff and theta_z_diff


    def publish_setpoints(self,pose):
        roll, pitch, yaw = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        #self.pub_x.Publish(Float64(pose.position.x))
        #self.pub_y.Publish(Float64(pose.position.y))
        self.pub_z.Publish(Float64(pose.position.y))
        self.pub_theta_x.Publish(Float64(roll))
        self.pub_theta_y.Publish(Float64(pitch))
        self.pub_theta_z.Publish(Float64(yaw))





    
    def move_position(self, value):
        
        self.depth = value.data


if __name__ == "__main__":
    rospy.init_node("state_control_action_server")
    s = StateControlActionServer()
    rospy.spin()


