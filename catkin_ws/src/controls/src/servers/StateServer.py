from BaseServer import BaseServer
import actionlib
from geometry_msgs.msg import Point
from auv_msgs.msg import StateAction, StateFeedback, StateResult

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

