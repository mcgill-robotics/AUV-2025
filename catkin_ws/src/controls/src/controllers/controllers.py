import rospy

from std_msgs.msg import Float64


class Controller:

    Controller.si_topic_dict = {'HEAVE':'heave', 'SURGE':'surge'}

    __init__(self):

    def controller_cb(self):
        pass

    
    def publish(self):
        pass

    def release(self):
        

class DirectController(Controller):

    __init__(self, cmd):
        self.var  = cmd.var
        self.type = 'DIRECT'
        self.effort = cmd.effort

        self.timer = rospy.Timer(rospy.Duration(cmd.duration), self.reset_effort)
        

        # publishing to superimposer
        si_topic = Controller.si_topic_dict[self.var]
        self.pub_si = rospy.Publisher(si_topic, Float64, queue_size=50)


    def new_command(self, cmd):
        self.effort = cmd.effort
        self.timer = rospy.Timer(rospy.Duration(cmd.duration), self.reset_effort)


    def reset_effort(self):
        self.effort = 0.0

    def release(self):
        # update router allocation table
        # status depends on timer
        status = result = True
        return status, result



    def publish(self):
        self.pub_si.publish(self.effort)


class StateController(Controller):

    __init__(self, cmd):
        self.var  = cmd.var
        self.type = 'STATE'
        self.effort = None
        self.setpoint = cmd.setpoint
        

        # publishing to superimposer
        si_topic = Controller.si_topic_dict[self.var]
        self.pub_si = rospy.Publisher(si_topic, Float64, queue_size=50)

        # pid
        pid_setpoint_topic = self.var + '_setpoint'
        pid_effort_topic = self.var + '_effort'
        pid_state_topic = self.var + '_state'

        self.pub_setpoint = rospy.Publisher(pid_setpoint_topic, Float64, queue_size=50)
        sub_effort = rospy.Subscriber(pid_effort_topic, Float64, self.effort_cb, queue_size=50)

        # set setpoint - TODO - do outside of constructor
        self.pub_setpoint.publish(self.setpoint)


    def new_command(self, cmd):
        self.effort = None
        self.setpoint = cmd.setpoint
        self.pub_setpoint.publish(self.setpoint)


    def effort_cb(self, effort):
        self.effort = effort


    def publish(self):
        self.pub_si.publish(self.effort)

