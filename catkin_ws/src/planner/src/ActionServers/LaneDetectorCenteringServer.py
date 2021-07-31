import rospy
import actionlib
import math

from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Point
from cv.msg import CvTarget
from planner.msg import LaneDetectorCenteringAction, LaneDetectorCenteringFeedback, LaneDetectorCenteringResult

class LaneDetectorCenteringServer():

    def __init__(self):

        # Constants
        # self.VIEWFRAME_CENTER_Y     = None
        # self.VIEWFRAME_CENTER_X     = None
        # self.RADIAL_THRESHOLD       = None
        self.COUNTS_FOR_STABILITY   = 2 # This should be higher, but we are testing...

        # Data variables
        self.current_stable_counts  = 0

        # Publishers to PIDs (we need a different PID for every direction (x, y))

        # Sway PID
        self.sway_pid_enable_pub     = rospy.Publisher('/centroid_x_pid/enable', Bool, queue_size = 1)
        self.sway_pid_setpoint_pub   = rospy.Publisher('/centroid_x_pid/setpoint', Float64, queue_size = 1)
        self.delta_x_pub             = rospy.Publisher('/lane_detector/centroid_delta_x', Float64, queue_size = 1)
        # delta_x is "state data" for sway PID

        # Surge PID
        self.surge_pid_enable_pub    = rospy.Publisher('/centroid_y_pid/enable', Bool, queue_size = 1)
        self.surge_pid_setpoint_pub  = rospy.Publisher('/centroid_y_pid/setpoint', Float64, queue_size = 1)
        self.delta_y_pub             = rospy.Publisher('/lane_detector/centroid_delta_y', Float64, queue_size = 1)
        # delta_y is "state data" for surge PID

        # Define the action server, and start it
        self._action_name = 'LDCentering'
        self._as = actionlib.SimpleActionServer(self._action_name, LaneDetectorCenteringAction,
                                                     execute_cb=self.execute_cb, auto_start = False)
        self._as.start()


    def centroid_loc_cb(self, cvmsg):
        
        point = cvmsg.gravity
        #print('Reached callback')
        y_dist_to_center = point.y - self.VIEWFRAME_CENTER_Y
        x_dist_to_center = point.x - self.VIEWFRAME_CENTER_X
        self.distance_centroid_to_center = math.sqrt(y_dist_to_center * y_dist_to_center
                                                    + x_dist_to_center * x_dist_to_center)

        self.delta_y_pub.publish(y_dist_to_center)
        self.delta_x_pub.publish(x_dist_to_center)
        #print('Centroid distance from center: {}'.format(self.distance_centroid_to_center))

        if (self.distance_centroid_to_center < self.RADIAL_THRESHOLD):
                self.current_stable_counts += 1
        else:
                self.current_stable_counts = 0

        return


    def execute_cb(self, goal):
        rospy.loginfo('Executing Lane Detector Centering Server')

        #Set a variable that will remain true unless the server is pre-empted
        success = True
        
        print(goal)

        # Unpack the goal message into its components
        self.VIEWFRAME_CENTER_Y = goal.image_center_point.y
        self.VIEWFRAME_CENTER_X = goal.image_center_point.x
        self.RADIAL_THRESHOLD = 0.3 * self.VIEWFRAME_CENTER_Y * 2

        # This is bad practice, but avoids a logical flaw when the subscriber gets to the callback before the VIEWFRAME_CENTER_Y is initialized
        self.centroid_sub = rospy.Subscriber('cv/down_cam_target_centroid', CvTarget, self.centroid_loc_cb)

        # Set the setpoints for the surge and sway PIDs
        self.surge_pid_setpoint_pub.publish(0)
        self.sway_pid_setpoint_pub.publish(0)
        #Then enable the pids!
        self.surge_pid_enable_pub.publish(True)
        self.sway_pid_enable_pub.publish(True)

        while not (self.current_stable_counts >= self.COUNTS_FOR_STABILITY):

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            remaining_counts = self.COUNTS_FOR_STABILITY - self.current_stable_counts
            rospy.loginfo_throttle(1, 'Moving towards centroid: Need {} more stable readings'.format(remaining_counts))
            self._as.publish_feedback(LaneDetectorCenteringFeedback(stable_count_centroid_remaining = remaining_counts))

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(result = LaneDetectorCenteringResult(Point(x=0, y=0))) # This is awful and hardcoded, please do what you want with it

        # We should turn off PIDs no matter if we succeed or not
        print("Turning off surge and sway PIDs")
        self.surge_pid_enable_pub.publish(False)
        self.sway_pid_enable_pub.publish(False)

        return


if __name__ == '__main__':
    rospy.init_node('LaneDetectorCenteringServer')
    server = LaneDetectorCenteringServer()
    rospy.spin()   