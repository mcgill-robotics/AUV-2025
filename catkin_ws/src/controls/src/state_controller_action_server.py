#! /usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose
from auv_msgs.msg import StateAction, StateFeedback, StateResult
from std_msgs.msg import Float64

class StateControlActionServer():

    def __init__(self) -> None:
        self.server = actionlib.SimpleActionServer('state_control_action', StateAction, execute_cb= self.callback, auto_start = False)
        self.feedback = StateFeedback()
        self.result = StateResult()
        self.pub = rospy.Publisher('theta_z_setpoint', Float64, queue_size=50)
        self.server.start()

    def callback(self, goal):
        rate = rospy.Rate(1)
        success = True
        depth = Pose.position.z

        rospy.loginfo('%s: Executing State Action Controller')

        while(depth < goal.setpoint - 0.5 or depth > goal.setpoint + 0.5):

            if self.server.is_preempt_requested():
                rospy.loginfo("Preempted")
                self.server.set_preempted()
                success = False
                self.feedback.status = False
                break

            depth = rospy.Subscriber('state_theta_z', Float64, self.move_position)
            self.pub.publish(depth)
            self.server.publish_feedback(depth)
            
            rate.sleep()

            if success:
                self.result = self.feedback
                rospy.loginfo("Succeeded")
                self.server.set_succeeded(self.result)

    
    def move_position(self):
        
        return Pose.position.z


if __name__ == "__main__":
    rospy.init_node("state_control_action_server")
    s = StateControlActionServer()
    rospy.spin()


