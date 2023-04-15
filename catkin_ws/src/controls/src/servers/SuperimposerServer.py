import rospy
import actionlib
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from auv_msgs.msg import SuperimposerAction, SuperimposeFeedback, SuperimposerResult
from std_msgs.msg import Float64
import time



class StateServer():

    def __init__(self) -> None:
        self.server = actionlib.SimpleActionServer('superimposer_server', SuperimposerAction, execute_cb= self.callback, cancel_cb=self.cancel, auto_start = False)
        #self.feedback = StateFeedback()
        #self.result = StateResult()

        self.pub_x = rospy.Publisher('surge', Float64, queue_size=50)
        self.pub_y = rospy.Publisher('sway', Float64, queue_size=50)
        self.pub_z = rospy.Publisher('heave', Float64, queue_size=50)
        self.pub_theta_x = rospy.Publisher('roll', Float64, queue_size=50)
        self.pub_theta_y = rospy.Publisher('pitch', Float64, queue_size=50)
        self.pub_theta_z = rospy.Publisher('yaw', Float64, queue_size=50)
        
        self.server.start()



    def callback(self, goal):

        self.goal = goal
        #unset pids
        if(goal.include_zeros or goal.effort.force.x != 0):
            #unset pids
            self.pub_x.publish(goal.effort.force.x)
        if(goal.include_zeros or goal.effort.force.y != 0):
            #unset pids
            self.pub_y.publish(goal.effort.force.y)
        if(goal.include_zeros or goal.effort.force.z != 0):
            #unset pids
            self.pub_z.publish(goal.effort.force.z)

        if(goal.include_zeros or goal.effort.torque.x != 0):
            #unset pids
            self.pub_theta_x.publish(goal.effort.torque.x)
        if(goal.include_zeros or goal.effort.torque.y != 0):
            #unset pids
            self.pub_theta_y.publish(goal.effort.torque.y)
        if(goal.include_zeros or goal.effort.torque.x != 0):
            #unset pids
            self.pub_theta_z.publish(goal.effort.torque.z)

        fb = SuperimposeFeedback()
        fb.moving = True
        self.server.publish_feedback(fb)
    
    def cancel(self):
        if(self.goal.include_zeros or self.goal.effort.force.x != 0):
            #unset pids
            self.pub_x.publish(Float64(0))
        if(self.goal.include_zeros or self.goal.effort.force.y != 0):
            #unset pids
            self.pub_y.publish(Float64(0))
        if(self.goal.include_zeros or self.goal.effort.force.z != 0):
            #unset pids
            self.pub_z.publish(Float64(0))

        if(self.goal.include_zeros or self.goal.effort.torque.x != 0):
            #unset pids
            self.pub_theta_x.publish(Float64(0))
        if(self.goal.include_zeros or self.goal.effort.torque.y != 0):
            #unset pids
            self.pub_theta_y.publish(Float64(0))
        if(self.goal.include_zeros or self.goal.effort.torque.x != 0):
            #unset pids
            self.pub_theta_z.publish(Float64(0))

        # result = SuperimposerResult()
        #self.server.set_preempted

    