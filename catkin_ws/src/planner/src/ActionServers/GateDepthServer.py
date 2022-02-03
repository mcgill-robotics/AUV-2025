import rospy
import actionlib
import math

from std_msgs.msg import Float64, Bool
from planner.msg import DepthAction, DepthFeedback, DepthResult

class GateDepthServer():

    def __init__(self):

        # Constants
        self.COUNTS_FOR_STABILITY   = 2 # This should be higher, but we are testing...

        # Constants from goal
        self.TARGET_DEPTH           = 0 # meters
        self.DEPTH_THRESHOLD        = 0 # meters

        # Data variables
        self.stable_counts          = 0
        self.stable_at_depth        = False
        self.current_depth          = 0

        # Depth subscriber
        self.depth_sub              = rospy.Subscriber('/state_estimation/depth', Float64, self.depth_cb)

        # Depth PID publishers
        self.depth_pid_enable_pub   = rospy.Publisher('/controls/depth_pid/pid_enable' , Bool    , queue_size=1)
        self.depth_pid_setpoint_pub = rospy.Publisher('/controls/depth_pid/setpoint'   , Float64 , queue_size=1)
        # Depth PID gets "data" from depth sensor, no need to publish from here

        # Define the action server, and start it
        self._action_name = 'Depth'
        self._as = actionlib.SimpleActionServer(self._action_name, DepthAction,
                                                     execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        

    def depth_cb(self, depth_msg):
        
        self.current_depth = depth_msg.data

        if abs(self.current_depth - self.TARGET_DEPTH) < self.DEPTH_THRESHOLD :
            self.stable_counts += 1
        else:
            self.stable_counts = 0

        return


    def execute_cb(self, goal):
        rospy.loginfo('Achieving depth')

        # Set a variable that will remain true unless the server is pre-empted
        success = True
        
        print("Goal:")
        print(goal)

        # Unpack the goal message into its components
        self.TARGET_DEPTH = goal.depth_target.data
        self.DEPTH_THRESHOLD = 0.1 * self.TARGET_DEPTH

        # Set the setpoints for the surge and sway PIDs
        self.depth_pid_setpoint_pub.publish(self.TARGET_DEPTH)
        # Then enable the pids!
        self.depth_pid_enable_pub.publish(True)
        # Reset stable counts
        self.stable_counts = 0

        while not (self.stable_counts >= self.COUNTS_FOR_STABILITY):

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            remaining_counts = self.COUNTS_FOR_STABILITY - self.stable_counts
            rospy.loginfo_throttle(1, 'Going to depth: Need {} more stable readings'.format(remaining_counts))
            self._as.publish_feedback(DepthFeedback(current_depth = Float64(self.current_depth)))

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(result = DepthResult(current_depth = Float64(self.current_depth)))
        else:
            # We should turn off PIDs only if we do not succeed (PID will keep running in the background and we can publish new setpoints as we please)
            print("Did not reach depth, turning off heave PID")
            self.depth_pid_enable_pub.publish(False)

        return


if __name__ == '__main__':
    rospy.init_node('GateDepthServer')
    server = GateDepthServer()
    rospy.spin()   