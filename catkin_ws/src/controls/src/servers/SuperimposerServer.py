from BaseServer import BaseServer
import actionlib
from auv_msgs.msg import SuperimposerAction, SuperimposerFeedback, SuperimposerGoal, SuperimposerResult
from std_msgs.msg import Float64, Bool

class SuperimposerServer(BaseServer):
    def __init__(self):
        super().__init__()
        self.establish_pid_enable_publishers()
        self.establish_pid_publishers()
        self.establish_state_subscribers()

        self.pub_x_effort = None
        self.pub_y_effort = None
        self.pub_z_effort = None
        self.pub_theta_x_effort = None
        self.pub_theta_y_effort = None
        self.pub_theta_z_effort = None

    
    def displace_goal(self,displace):
        print("displacing goal")
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
    
    def cancel(self):
        if(self.goal.do_surge.data):
            self.pub_x_effort.publish(0)
        if(self.goal.do_sway.data):
            self.pub_y_effort.publish(0)
        if(self.goal.do_heave.data):
            self.pub_z_effort.publish(0)
        if(self.goal.do_roll.data):
            self.pub_theta_x_effort.publish(0)
        if(self.goal.do_pitch.data):
            self.pub_theta_y_effort.publish(0)
        if(self.goal.do_yaw.data):
            self.pub_theta_z_effort.publish(0)

    
    def callback(self, goal):
        print("received new goal")
        print(goal)
        if(goal.displace.data):
            self.goal = self.displace_goal(goal)
        else:
            self.goal = goal
        #unset pids
        if(goal.do_surge.data):
            self.pub_x_enable.publish(Bool(False))
            self.pub_x_effort.publish(self.goal.effort.force.x)
        if(goal.do_sway.data):
            #unset pids
            self.pub_y_enable.publish(Bool(False))
            self.pub_y_effort.publish(self.goal.effort.force.y)
        if(goal.do_heave.data):
            #unset pids
            self.pub_z_enable.publish(Bool(False))
            self.pub_z_effort.publish(self.goal.effort.force.z)
        if(goal.do_roll.data):
            #unset pids
            self.pub_theta_x_enable.publish(Bool(False))
            self.pub_theta_x_effort.publish(self.goal.effort.torque.x)
        if(goal.do_pitch.data):
            #unset pids
            self.pub_theta_y_enable.publish(Bool(False))
            self.pub_theta_y_effort.publish(self.goal.effort.torque.y)
        if(goal.do_yaw.data):
            #unset pids
            self.pub_theta_z_enable.publish(Bool(False))
            self.pub_theta_z_effort.publish(self.goal.effort.torque.z)

        self.server.set_succeeded()



class LocalSuperimposerServer(SuperimposerServer):
    def __init__(self):
        super().__init__()
        self.server = actionlib.SimpleActionServer('superimposer_local_server', SuperimposerAction, execute_cb= self.callback, auto_start = False)


        self.pub_x_effort = rospy.Publisher('surge', Float64, queue_size=50)
        self.pub_y_effort = rospy.Publisher('sway', Float64, queue_size=50)
        self.pub_z_effort = rospy.Publisher('heave', Float64, queue_size=50)
        self.pub_theta_x_effort = rospy.Publisher('roll', Float64, queue_size=50)
        self.pub_theta_y_effort = rospy.Publisher('pitch', Float64, queue_size=50)
        self.pub_theta_z_effort = rospy.Publisher('yaw', Float64, queue_size=50)
  
        self.server.start()

class GlobalSuperimposerServer(SuperimposerServer):
    def __init__(self):
        super().__init__()
        self.server = actionlib.SimpleActionServer('superimposer_global_server', SuperimposerAction, execute_cb= self.callback, auto_start = False)

        self.pub_x_effort = rospy.Publisher('global_x', Float64, queue_size=50)
        self.pub_y_effort = rospy.Publisher('global_y', Float64, queue_size=50)
        self.pub_z_effort = rospy.Publisher('global_z', Float64, queue_size=50)
        self.pub_theta_x_effort = rospy.Publisher('roll', Float64, queue_size=50)
        self.pub_theta_y_effort = rospy.Publisher('pitch', Float64, queue_size=50)
        self.pub_theta_z_effort = rospy.Publisher('yaw', Float64, queue_size=50)

        self.server.start()
    