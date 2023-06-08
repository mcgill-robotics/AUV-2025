from BaseServer import BaseServer
import actionlib
from geometry_msgs.msg import Point
from auv_msgs.msg import StateAction, StateFeedback, StateResult


"""
This server class executes a state goal, which has a goal pose to enter.
The goal pose can be considered a global position, or a displacement within
the world reference frame.
"""
class StateServer(BaseServer):
    def __init__(self):
        super().__init__()
        self.establish_pid_publishers()
        self.establish_pid_enable_publishers()
        self.establish_state_subscribers()
        self.server = actionlib.SimpleActionServer('state_server', StateAction, execute_cb= self.callback, auto_start = False)
        self.server.start()

    def callback(self, goal):
        """
        Execute callback for this server. Executes the goal.
        If the goal is a displace, calculates a global pose to move to.
        Sets the pids then waits for the auv to enter the pose before
        setting the state to succeeded.
        """
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
        """
        Takes in a displacement goal pose and returns the corresponding
        world frame pose.
        """
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

