import rospy

import actionlib

from auv_msgs.msg import DirectAction, StateAction
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Pose


class Controller:

    PUB_PERIOD = 1 # s 
    si_topic_for_dof = {'HEAVE':'heave', 'SWAY':'sway', 'SURGE':'surge'}

    def __init__(self, dof):
        self.dof = dof

        # for publishing to superimposer
        si_topic = Controller.si_topic_for_dof[dof]
        self.pub_si = rospy.Publisher(si_topic, Float64, queue_size=50)


    # this is a hack to use as callback to timer
    def publish_cb(self, timer):
        self.publish_to_si()

    def publish_to_si(self):
        print("publishing to si")
        self.pub_si.publish(self.effort)

    def reset_effort(self):
        self.effort = 0.0
        self.publish_to_si()

    def start(self):
        self.action_server.start()

    def finish(self):
        self.action_server = None


class DirectController(Controller):

    def __init__(self, dof):
        super().__init__(dof)
        self.type = 'DIRECT'

        # set up action server
        self.action_server = actionlib.SimpleActionServer('controller/' + dof, DirectAction, self.goal_cb, False)

    def goal_cb(self, cmd):
        print("controller goal cb - effort", cmd.effort)
        self.effort = cmd.effort
        self.duration = cmd.duration

        # controller execution/publishing to superimposer
        self.pub_timer = rospy.Timer(rospy.Duration(Controller.PUB_PERIOD), self.publish_cb)

        # after <duration> has elapsed, publish reset and return action result
        rospy.sleep(self.duration) #TODO - check whether this is blocking
        self.pub_timer.shutdown() # stop publishing periodically
        self.reset_effort()

        self.action_server.set_succeeded() #TODO - what to return to client?


    # TODO - make sure this is blocking
    def finish(self):
        self.pub_timer.shutdown() # stop publishing periodically
        self.reset_effort()
        # TODO: status depends on timer
        self.action_server.set_aborted() # TODO - check - return time spent 
        super().finish() #TODO - probably doesn't matter
        return


class StateController(Controller):
    THRESHOLD = 0.05 # m
    REQUIRED_COUNT = 10 #TODO: use timer to avoid coupling to state rate

    def __init__(self, dof):
        super().__init__(dof)
        self.type = 'STATE'
        self.threshold_count = 0 

        # set up action server
        self.action_server = actionlib.SimpleActionServer('controller/' + dof, StateAction, self.goal_cb, False)
        
        # pid
        # TODO: dynamically create pid nodes 
        pid_enable_topic = self.direction + '_enable'
        pid_setpoint_topic = self.direction + '_setpoint'
        pid_effort_topic = self.direction + '_effort'
        pid_state_topic = self.direction + '_state'

        self.pub_enable = rospy.Publisher(pid_enable_topic, Bool, queue_size=50)
        self.pub_setpoint = rospy.Publisher(pid_setpoint_topic, Float64, queue_size=50)
        self.sub_effort = rospy.Subscriber(pid_effort_topic, Float64, self.pid_effort_cb, queue_size=50)

        # this is to check if/when we have reached the state
        self.sub_state = rospy.Subscriber('state', Pose, self.state_cb)


    def goal_cb(self, cmd):
        self.effort = 0.0
        self.setpoint = cmd.setpoint
        self.maintain = cmd.maintain

        self.pub_setpoint.publish(self.setpoint)

        # TODO - wait until effort is published - just set init. effort to 0.0?
        # controller execution/publishing to superimposer
        self.pub_timer = rospy.Timer(rospy.Duration(Controller.PUB_PERIOD), self.publish_cb)
        return


    def pid_effort_cb(self, effort):
        self.effort = effort


    def state_cb(self, pose):
        if abs(state - self.setpoint) < StateController.THRESHOLD:
            # state achieved
            self.threshold_count += 1
            if self.threshold_count >= StateController.REQUIRED_COUNT:
                self.state_reached()
        else:
            self.threshold_count = 0


    def state_reached(self):
        if not self.maintain:
            # TODO: update alloc_table return result (use result_cb)
            self.action_server.set_succeeded()


    def finish(self):
        self.pub_enable.publish(False)
        super().finish()
