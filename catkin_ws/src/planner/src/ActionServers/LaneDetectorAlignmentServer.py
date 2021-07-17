import rospy
import actionlib
import math
from std_msgs.msg import Bool, Float64
from cv.msg import CvTarget
from planner.msg import LaneDetectorAlignmentAction, LaneDetectorAlignmentFeedback, LaneDetectorAlignmentResult
from geometry_msgs.msg import Point

class LaneDetectorAlignmentServer():
    def __init__(self):


        #self.TARGET_ANGLE = 0
        self.COUNTS_FOR_STABILITY   = 2 # This should be higher, but we are testing...
        self.current_stable_counts  = 0
        self.ANGLE_THRESHOLD       = 20*3.14/180 #Chosen for testing 

        # Enabling PIDs
        self.yaw_pid_enable_pub          = rospy.Publisher('/lane_yaw_pid/enable', Bool, queue_size = 1)
        # Setpoints for PIDs
        self.yaw_pid_setpoint_pub        = rospy.Publisher('/lane_yaw_pid/setpoint', Float64, queue_size = 1)
        # Publishing "data" to PIDs
        self.current_angle_pub = rospy.Publisher('/cv/down_cam_heading_Hough', Float64, queue_size = 1)

        # Define the action server, and start it
        self._action_name = 'LDAlignment'
        self._as = actionlib.SimpleActionServer(self._action_name, LaneDetectorAlignmentAction,
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
        
        # Set the setpoints for the surge and sway PIDs
        self.yaw_pid_setpoint_pub.publish(self.TARGET_ANGLE)
      
        #Then enable the pids!
        self.yaw_pid_enable_pub.publish(True)

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
            self._as.set_succeeded(result = LaneDetectorAlignmentResult(0.0)) # This is awful and hardcoded, please do what you want with it

    def heading_align_cb(self, angle):
        #print('Reached callback')

        self.current_angle_pub.publish(angle)
     
        #print('Centroid distance from center: {}'.format(self.distance_centroid_to_center))

        if (angle < self.ANGLE_THRESHOLD):
                self.current_stable_counts += 1
        else:
                self.current_stable_counts = 0


if __name__ == '__main__':
    rospy.init_node('LaneDetectorAlignmentServer')
    server = LaneDetectorAlignmentServer()
    rospy.spin()   