#! /usr/bin/python3

import rospy
from BaseServer import BaseServer
import actionlib
from auv_msgs.msg import SuperimposerAction, SuperimposerFeedback, SuperimposerGoal, SuperimposerResult
from std_msgs.msg import Float64, Bool


"""
Abstract class for the superimposer servers. The superimposer servers
accept a SuperimposerGoal which has an effort to exert on the auv
via the thrusters. The preempt callback is the default cancel method
which sets the pids to the current position.
"""
class SuperimposerServer(BaseServer):
    def __init__(self):
        super().__init__()
        self.establish_pid_enable_publishers()
        self.establish_pid_publishers()
        self.establish_state_subscribers()

        self.pub_surge = rospy.Publisher('surge', Float64, queue_size=50)
        self.pub_sway = rospy.Publisher('sway', Float64, queue_size=50)
        self.pub_heave = rospy.Publisher('heave', Float64, queue_size=50)

        self.pub_roll = rospy.Publisher('roll', Float64, queue_size=50)
        self.pub_pitch = rospy.Publisher('pitch', Float64, queue_size=50)
        self.pub_yaw = rospy.Publisher('yaw', Float64, queue_size=50)

    
    def displace_goal(self,displace):
        """
        Accepts a displacement goal effort, and adds
        it to the current goal effort. If the goal effort
        does not exist it is treated as 0.
        """
        if(self.goal == None):
            return displace

        new_goal = displace
        new_goal.effort.force.x += self.goal.effort.force.x
        new_goal.effort.force.y += self.goal.effort.force.y
        new_goal.effort.force.z += self.goal.effort.force.z
        new_goal.effort.torque.x += self.goal.effort.torque.x
        new_goal.effort.torque.y += self.goal.effort.torque.y
        new_goal.effort.torque.z += self.goal.effort.torque.z
        return new_goal

    
    def callback(self, goal):
        """
        Executes a Superimposer Goal. Sets the efforts to the goal efforts.
        """
        print("received new goal")
        print(goal)
        if(goal.displace.data):
            self.goal = self.displace_goal(goal)
        else:
            self.goal = goal
        

        if(self.goal.do_x.data):
            self.pub_surge.publish(self.goal.effort.force.x)
        if(self.goal.do_y.data):
            self.pub_sway.publish(self.goal.effort.force.y)
        if(self.goal.do_z.data):
            self.pub_heave.publish(self.goal.effort.force.z)

        
        if(self.goal.do_roll.data):
            self.pub_theta_x_effort.publish(self.goal.effort.torque.x)
        if(self.goal.do_pitch.data):
            self.pub_theta_y_effort.publish(self.goal.effort.torque.y)
        if(self.goal.do_yaw.data):
            self.pub_theta_z_effort.publish(self.goal.effort.torque.z)

        self.server.set_succeeded()

    def unset_pids(self,goal):
        """
        Unsets the pids for the given goal.   
        """
        if(goal.do_roll.data):
            self.pub_theta_x_enable.publish(0)
        if(goal.do_pitch.data):
            self.pub_theta_y_enable.publish(0)
        if(goal.do_yaw.data):
            self.pub_theta_z_enable.publish(0)

        # ANTOINE TODO: determine which pids to unset when recieving a superimposer goal

        # if(goal.do_surge.data or goal.do_sway.data):
        #     self.pub_x_enable.publish(0)
        #     self.pub_y_enable.publish(0)
        # if(goal.do_heave.data):
        #     self.pub_z_enable.publish(0)

    