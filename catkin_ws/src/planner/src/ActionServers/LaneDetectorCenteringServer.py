import rospy
import actionlib
import math
from std_msgs.msg import Bool, Float64
from cv.msg import CvTarget
from planner.msg import LaneDetectorCenteringAction

class LaneDetectorCenteringServer():

    def __init__(self):
        # Define all of the publishers that we're going to need later on
        # Publishers to PIDs (we need a different PID for every direction (x, y))
        self.centroid_surge_pid_enable_pub          = rospy.Publisher('/centroid_y_pid/enable', Bool, queue_size = 1)
        self.centroid_surge_pid_setpoint_pub        = rospy.Publisher('/centroid_y_pid/setpoint', Float64, queue_size = 1)
        self.centroid_sway_pid_enable_pub           = rospy.Publisher('/centroid_x_pid/enable', Bool, queue_size = 1)
        self.centroid_sway_pid_setpoint_pub         = rospy.Publisher('/centroid_x_pid/setpoint', Float64, queue_size = 1)
        self.heading_lane_detector_yaw_enable_pub   = rospy.Publisher('/lane_yaw_pid/enable', Bool, queue_size = 1)
        self.heading_lane_detector_yaw_setpoint_pub = rospy.Publisher('/lane_yaw_pid/setpoint', Float64, queue_size = 1)

        #Define the action server, and start iot
        self._action_name = 'LDCentering'
        self._as = actionlib.SimpleActionServer(self._action_name, LaneDetectorCenteringAction,
                                                     execute_cb=self.execute_cb, auto_start = False)
        
        #self.centroid_sub = rospy.Subscriber('cv/down_cam_target_centroid', CvTarget, self.centroid_loc_cb)
        self._as.start()
        

    def execute_cb(self, goal):
        rospy.loginfo('Executing Lane Detector Centering Server')

        #Set a variable that will remain true unless the server is pre-empted
        success = True
        
        print(goal)
        #Unpack the goal message into its components
        self.VIEWFRAME_CENTER_X = goal.image_center_point.x
        self.VIEWFRAME_CENTER_Y = goal.image_center_point.y

        # Set the setpoints for the surge and sway PIDs
        self.centroid_surge_pid_setpoint_pub.publish(self.VIEWFRAME_CENTER_Y)
        self.centroid_sway_pid_setpoint_pub.publish(self.VIEWFRAME_CENTER_X)
        #Then enable the pids!
        self.centroid_surge_pid_enable_pub.publish(True)
        self.centroid_sway_pid_enable_pub.publish(True)

        while not (self.current_stable_counts_centroid >= self.COUNTS_FOR_STABILITY):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            remaining_counts = self.COUNTS_FOR_STABILITY - self.current_stable_counts_centroid
            rospy.loginfo_throttle(1, 'Moving towards centroid: Need {} more stable readings'.format(remaining_counts))
            
            self._as.publish_feedback(remaining_counts)

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded()  

    def centroid_loc_cb(self, cvmsg):
        point = cvmsg.gravity
        #print('Reached callback')
        center_y_dist_to_centroid = point.y - self.VIEWFRAME_CENTER_Y
        center_x_dist_to_centroid = point.x - self.VIEWFRAME_CENTER_X
        self.distance_centroid_to_center = math.sqrt(center_y_dist_to_centroid * center_y_dist_to_centroid\
                                                    + center_x_dist_to_centroid * center_x_dist_to_centroid)

        self.centroid_delta_y_pub.publish(center_y_dist_to_centroid)
        self.centroid_delta_x_pub.publish(center_x_dist_to_centroid)
        #print('Centroid distance from center: {}'.format(self.distance_centroid_to_center))

        if (self.distance_centroid_to_center < self.VIEWFRAME_CENTROID_RADIAL_THRESHOLD_TO_CENTER):
                self.current_stable_counts_centroid += 1
        else:
                self.current_stable_counts_centroid = 0

if __name__ == '__main__':
    rospy.init_node('LaneDetectorCenteringServer')
    server = LaneDetectorCenteringServer()
    rospy.spin()   