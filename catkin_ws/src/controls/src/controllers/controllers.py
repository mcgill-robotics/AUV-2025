import rospy

from std_msgs.msg import Float64, Bool


class Controller:

    Controller.si_topic_dict = {'HEAVE':'heave', 'SWAY':'sway', 'SURGE':'surge'}

    __init__(self):
        self.direction  = cmd.direction
        self.type = cmd.ctl_type

        # publishing to superimposer
        si_topic = Controller.si_topic_dict[self.direction]
        self.pub_si = rospy.Publisher(si_topic, Float64, queue_size=50)

    def publish(self):
        pass

    def update_command(self, cmd):
        pass

    def start(self):
        pass

    def destroy(self):
        pass
        

class DirectController(Controller):

    __init__(self, cmd):
        super().__init__(cmd)
        self.effort = cmd.effort
        self.duration = cmd.duration


    def update_command(self, cmd):
        self.effort = cmd.effort
        self.duration = cmd.duration
        self.start()

    def reset_effort(self):
        self.effort = 0.0


    def publish(self):
        self.pub_si.publish(self.effort)


    def start(self):
        self.timer = rospy.Timer(rospy.Duration(self.duration), self.reset_effort)

    def stop(self):
        # update router allocation table
        # TODO: status depends on timer
        status = result = True
        return status, result

class StateController(Controller):

    __init__(self, cmd):
        super().__init__(cmd)
        self.effort = None
        self.setpoint = cmd.setpoint
        self.threshold = 0.05 # m
        self.maintain = cmd.maintain
        
        # pid
        # TODO: dynamically create pid nodes 
        pid_enable_topic = self.direction + '_enable'
        pid_setpoint_topic = self.direction + '_setpoint'
        pid_effort_topic = self.direction + '_effort'
        pid_state_topic = self.direction + '_state'

        self.pub_enable = rospy.Publisher(pid_enable_topic, Bool, queue_size=50)
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


    def state_cb(self, state):
        if abs(state - self.setpoint) < self.threshold:



    def publish(self):
        self.pub_si.publish(self.effort)

    
    def start(self):
        self.pub_enable.publish(True)


    def destroy(self):
        self.pub_enable.publish(False)
