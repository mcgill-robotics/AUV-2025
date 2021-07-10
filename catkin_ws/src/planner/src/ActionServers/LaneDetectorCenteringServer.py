import rospy
import actionlib
import math
from planner.msg import LaneDetectorCenteringFeedback, LaneDetectorCenteringResult

class LaneDetectorCenteringAction():
    feedback = LaneDetectorCenteringFeedback();
    result = LaneDetectorCenteringResult();

    def __init__(self):

        self._action_name = 'LaneDetectorCenteringServer'
        self._as = actionlib.SimpleActionServer(self._action_name, LaneDetectorCenteringAction,
                                                     execute_cb=self.execute_cb, auto_start = False)
        
        self.centroid_sub = rospy.Subscriber('cv/down_cam_target_centroid', CvTarget, self.centroid_loc_cb)
        self._as.start()
        

    def execute_cb(self, goal):
        rospy.loginfo('Executing state LaneDetector')
        success = True
        self.VIEWFRAME_CENTER_X = goal.image_center_point.x
        self.VIEWFRAME_CENTER_Y = goal.image_center_point.y

        # Centering on the lane
        self.centroid_surge_pid_setpoint_pub.publish(self.VIEWFRAME_CENTER_Y)
        self.centroid_sway_pid_setpoint_pub.publish(self.VIEWFRAME_CENTER_X)

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
    server = LaneDetectorCenteringAction()
    rospy.spin()   