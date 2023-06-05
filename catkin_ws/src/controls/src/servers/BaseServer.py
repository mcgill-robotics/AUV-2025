#! /usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from auv_msgs.msg import SuperimposerAction, SuperimposerFeedback, SuperimposerGoal, SuperimposerResult, StateAction, StateFeedback, StateResult
from std_msgs.msg import Float64, Bool
import time


class BaseServer():

    def __init__(self) -> None:
        print("starting server")
        self.cancelled = False
        self.goal = None
        


    def establish_pid_publishers(self):
        self.pub_z_pid = rospy.Publisher('z_setpoint', Float64, queue_size=50)
        self.pub_y_pid = rospy.Publisher('y_setpoint', Float64, queue_size=50)
        self.pub_x_pid = rospy.Publisher('x_setpoint', Float64, queue_size=50)
        self.pub_theta_x_pid = rospy.Publisher('theta_x_setpoint', Float64, queue_size=50)
        self.pub_theta_y_pid = rospy.Publisher('theta_y_setpoint', Float64, queue_size=50)
        self.pub_theta_z_pid = rospy.Publisher('theta_z_setpoint', Float64, queue_size=50)
        

    def establish_pid_enable_publishers(self):
        self.pub_x_enable = rospy.Publisher('pid_x_enable', Bool, queue_size=50)
        self.pub_y_enable = rospy.Publisher('pid_y_enable', Bool, queue_size=50)
        self.pub_z_enable = rospy.Publisher('pid_z_enable', Bool, queue_size=50)
        self.pub_theta_x_enable = rospy.Publisher('pid_theta_x_enable', Bool, queue_size=50)
        self.pub_theta_y_enable = rospy.Publisher('pid_theta_y_enable', Bool, queue_size=50)
        self.pub_theta_z_enable = rospy.Publisher('pid_theta_z_enable', Bool, queue_size=50)

    def establish_state_subscribers(self):
        self.position = Point(0,0,0)
        self.theta_x = 0
        self.theta_y = 0
        self.theta_z = 0
        self.sub = rospy.Subscriber("pose",Pose,self.set_position)
        self.sub = rospy.Subscriber("state_theta_x",Float64,self.set_theta_x)
        self.sub = rospy.Subscriber("state_theta_y",Float64,self.set_theta_y)
        self.sub = rospy.Subscriber("state_theta_z",Float64,self.set_theta_z)

    
    #callback for subscriber
    def set_position(self,data):
        #print("updated pose")
        self.position = data.position
    
    #callback for subscriber
    def set_theta_x(self,data):
        self.theta_x = data.data
    #callback for subscriber
    def set_theta_y(self,data):
        self.theta_y = data.data
    #callback for subscriber
    def set_theta_z(self,data):
        self.theta_z = data.data

    
    #generic cancel that publishes current position to pids to stay in place
    def cancel(self):
        self.cancelled = True
        self.pub_z_pid.publish(self.position.z)
        self.pub_y_pid.publish(self.position.y)
        self.pub_x_pid.publish(self.position.x)
        self.pub_theta_x_pid.publish(self.theta_x)
        self.pub_theta_y_pid.publish(self.theta_y)
        self.pub_theta_z_pid.publish(self.theta_z)
    
        # result = StateResult()
        # result.status = False
        self.server.set_succeeded()
    
    def enable_pids(self,goal):
        if(goal.do_x.data):
            self.pub_x_enable.publish(Bool(True))
        if(goal.do_y.data):
            self.pub_y_enable.publish(Bool(True))
        if(goal.do_z.data):
            self.pub_z_enable.publish(Bool(True))
        if(goal.do_theta_x).data:
            self.pub_theta_x_enable.publish(Bool(True))
        if(goal.do_theta_y.data):
            self.pub_theta_y_enable.publish(Bool(True))
        if(goal.do_theta_z.data):
            self.pub_theta_z_enable.publish(Bool(True))

        

    def wait_for_settled(self,position,rotation):
        interval = 1

        settled = False

        while not settled and not self.cancelled:
            #print("hi")
            start = time.time()
            while not self.cancelled and self.check_status(position,rotation):
                if(time.time() - start > interval):
                    settled = True
                    break
                rospy.sleep(0.01)

    #true if auv is in goal position
    def check_status(self,position,rotation):
        if(self.position == None or self.theta_x == None or self.theta_y == None or self.theta_z == None):
            return False

        tolerance_position = 0.5
        tolerance_orientation = 1

        x_diff = (not self.do_x) or abs(self.position.x - position.x) <= tolerance_position
        y_diff = (not self.do_y) or abs(self.position.y - position.y) <= tolerance_position
        z_diff = (not self.goal.do_z) or abs(self.position.z - position.z) <= tolerance_position
        theta_x_diff = (not self.goal.do_theta_x) or abs(self.theta_x - rotation.x) <= tolerance_orientation
        theta_y_diff = (not self.goal.do_theta_y) or abs(self.theta_y - rotation.y) <= tolerance_orientation
        theta_z_diff = (not self.goal.do_theta_z) or abs(self.theta_z - rotation.z) <= tolerance_orientation

        return x_diff and y_diff and z_diff and theta_x_diff and theta_y_diff and theta_z_diff


    def publish_setpoints(self, position, rotation):
        if (self.goal.do_x):
            self.pub_x_pid.publish(Float64(position.x))
        if (self.goal.do_y):
            self.pub_y_pid.publish(Float64(position.y))
        if(self.goal.do_z):
            self.pub_z_pid.publish(Float64(position.z))
        if(self.goal.do_theta_x):
            self.pub_theta_x_pid.publish(Float64(rotation.x))
        if(self.goal.do_theta_y):
            self.pub_theta_y_pid.publish(Float64(rotation.y))
        if(self.goal.do_theta_z):
            self.pub_theta_z_pid.publish(Float64(rotation.z))


class StateServer(BaseServer):
    def __init__(self):
        super().__init__()
        self.establish_pid_publishers()
        self.establish_pid_enable_publishers()
        self.establish_state_subscribers()
        self.server = actionlib.SimpleActionServer('state_server', StateAction, execute_cb= self.callback, auto_start = False)
        self.server.start()

    def callback(self, goal):
        print("got a message")
        # set the PIDs
        #print(goal.pose)
        self.goal = goal
        self.enable_pids(goal)
        if(goal.displace.data):
            goal_position, goal_rotation = self.dispalce_goal(goal)
        else:
            goal_position, goal_rotation = goal.position, goal.rotation
        print(goal_position,goal_rotation)
        self.publish_setpoints(goal_position,goal_rotation)

        # monitor when reached pose
        self.wait_for_settled(goal_position,goal_rotation)

        # rospy.loginfo("Succeeded")
        if(not self.cancelled):
            self.server.set_succeeded()
    
    def dispalce_goal(self, goal):
        goal_x = goal.position.x + self.position.x
        goal_y = goal.position.y + self.position.y
        goal_z = goal.position.z + self.position.z

        goal_theta_x = goal.rotation.x + self.theta_x
        goal_theta_y = goal.rotation.y + self.theta_y
        goal_theta_z = goal.rotation.z + self.theta_z

        goal_position = Point()
        goal_position.x = goal_x
        goal_position.y = goal_y
        goal_position.z = goal_z

        goal_rotation = Point()
        goal_rotation.x = goal_theta_x
        goal_rotation.y = goal_theta_y
        goal_rotation.z = goal_theta_z

        return goal_position, goal_rotation


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
    