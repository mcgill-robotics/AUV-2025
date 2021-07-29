import rospy
import actionlib

from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Point
from cv.msg import CvTarget
from planner.msg import LaneDetectorAlignmentAction, LaneDetectorAlignmentFeedback, LaneDetectorAlignmentResult

class LaneDetectorAlignmentServer():

    def __init__(self):

        # self.TARGET_ANGLE = 0 -> variable passed as goal, therefore set in execute_cb()
        self.COUNTS_FOR_STABILITY   = 2 # This should be higher, but we are testing...
        self.current_stable_counts  = 0
        self.ANGLE_THRESHOLD        = 20 * 3.14 / 180 # Chosen for testing (could be passed in goal
                                                        # along with the target angle using a
                                                        # custom message? In the future maybe?)

        # self.down_cam_heading_Hough = rospy.Subscriber('/cv/down_cam_heading_Hough',
            # Float64, self.heading_align_cb) -> subscriber set in execute_cb() because only want it
            # to start after self.TARGET_ANGLE is set. Only used for stable counts

        # Enabling PID
        self.yaw_pid_enable_pub     = rospy.Publisher('/lane_yaw_pid/enable', Bool, queue_size = 1)
        # Setpoints for PID
        self.yaw_pid_setpoint_pub   = rospy.Publisher('/lane_yaw_pid/setpoint', Float64, queue_size = 1)
        # No need to publish "data" to PID because it can read straight from CV without any data modification
        # (PID will read from same topic as the subscriber mentioned above)

        # Define the action server, and start it
        self._action_name   = 'LDAlignment'
        self._as            = actionlib.SimpleActionServer(self._action_name, LaneDetectorAlignmentAction,
                                                     execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):

        rospy.loginfo('Executing Lane Detector Alignment Server')

        #Set a variable that will remain true unless the server is pre-empted
        success = True
        
        print(goal)

        # Unpack the goal message into its components
        self.TARGET_ANGLE  = goal.image_angle_target

        # This is bad practice, but avoids a logical flaw when the subscriber gets to the callback before the VIEWFRAME_CENTER_Y is initialized
        self.down_cam_heading_Hough = rospy.Subscriber('/cv/down_cam_heading_Hough', Float64, self.heading_align_cb)
        
        # Set the setpoint
        self.yaw_pid_setpoint_pub.publish(self.TARGET_ANGLE)
        #Then enable the PID!
        self.yaw_pid_enable_pub.publish(True)

        # Stay stuck in loop until we are aligned and stable (maybe add timeout in future? This could be done from client)
        while not (self.current_stable_counts >= self.COUNTS_FOR_STABILITY):

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            remaining_counts = self.COUNTS_FOR_STABILITY - self.current_stable_counts
            rospy.loginfo_throttle(1, 'Aligning with lane: Need {} more stable readings'.format(remaining_counts))
            self._as.publish_feedback(LaneDetectorAlignmentFeedback(stable_count_angle_remaining = remaining_counts))

        if success:

            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(result = LaneDetectorAlignmentResult(Float64(0.0))) # This is awful and hardcoded, please do what you want with it

        return

    def heading_align_cb(self, cvmsg):

        angle = cvmsg.data

        if (angle < self.ANGLE_THRESHOLD):
                self.current_stable_counts += 1
        else:
                self.current_stable_counts = 0
        
        # We should turn off PID no matter if we succeed or not
        print("Turning off yaw PID")
        self.yaw_pid_enable_pub.publish(False)

        return

if __name__ == '__main__':
    rospy.init_node('LaneDetectorAlignmentServer')
    server = LaneDetectorAlignmentServer()
    rospy.spin()   