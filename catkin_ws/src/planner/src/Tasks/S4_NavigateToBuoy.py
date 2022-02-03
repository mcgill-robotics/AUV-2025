import rospy
import smach
import actionlib # No ActionServer implemented yet, we might want TODO this eventually
import math

from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Vector3Stamped, Point

class NavigateToBuoy(smach.State):
    # 0) Assume we are aligned with the lane right below us. The front camera should be able to see the buoy since it is white and big.
    # 1) Center the Buoy in the field of view using a PID on the convex hull
    # 2) Use the CV detector to report a probability that the current image in frame is the one we want at regular intervals
    # 3) If the probability exceeds a threshold, begin surging
    # END OF TASK
    # 4) Continue surging untill the convex hull takes up the whole field of vision, then continue for a hardcoded time (test!) to actually touch it
    # 5) Transition into the next task

    def __init__(self):

        # Initialize class as state and define transitions
        smach.State.__init__(self, outcomes=['BuoyReached'])

        # Abstract constants
        self.COUNTS_FOR_STABILITY = 2

        # Task constants
        self.VIEWFRAME_PIXEL_WIDTH                          = 720 # TODO testing # pixels
        self.VIEWFRAME_PIXEL_HEIGHT                         = 420 # TODO testing # pixels
        self.VIEWFRAME_CENTER_X                             = self.VIEWFRAME_PIXEL_WIDTH / 2.0
        self.VIEWFRAME_CENTER_Y                             = self.VIEWFRAME_PIXEL_HEIGHT / 2.0
        self.VIEWFRAME_CENTROID_RADIAL_THRESHOLD_TO_CENTER  = 0.05 * self.VIEWFRAME_PIXEL_HEIGHT # in pixels
        self.BUOY_TARGET_DISTANCE                           = 1 # in meters(?)
        self.BUOY_DISTANCE_THRESHOLD                        = 0.15

        # Variables
        self.stable_counts_centroid = 0
        self.stable_counts_distance = 0

        # Sway PID
        self.navigate_to_buoy_sway_pid_enable_pub = rospy.Publisher('/buoy_sway_pid/enable', Bool, queue_size = 1)
        self.navigate_to_buoy_sway_pid_setpoint_pub = rospy.Publisher('/buoy_sway_pid/setpoint', Float64, queue_size = 1)
        self.centroid_delta_x_pub = rospy.Publisher('/navigate_to_buoy/centroid_delta_x', Float64, queue_size = 1)
        # centroid_delta_x is "state data" for sway PID

        # Heave PID
        self.navigate_to_buoy_heave_pid_enable_pub = rospy.Publisher('/buoy_heave_pid/enable', Bool, queue_size = 1)
        self.navigate_to_buoy_heave_pid_setpoint_pub = rospy.Publisher('/buoy_heave_pid/setpoint', Float64, queue_size = 1)
        self.centroid_delta_y_pub = rospy.Publisher('/navigate_to_buoy/centroid_delta_y', Float64, queue_size = 1)
        # centroid_delta_y is "state data" for sway PID

        self.distance_to_buoy_pid_enable_pub = rospy.Publisher('/buoy_distance_pid/enable', Bool, queue_size = 1)
        self.distance_to_buoy_pid_setpoint_pub = rospy.Publisher('/buoy_distance_pid/setpoint', Float64, queue_size = 1)

        self.distance_to_buoy_sub = rospy.Subscriber('/cv/buoy_distance', Float64, self.distance_cb)


        self.target_sub = rospy.Subscriber('/cv/front_cam_target', Point, self.target_cb) # To put image centroid in middle of cam

    def target_cb(self, point):
        print('Reached callback')
        center_y_dist_to_centroid = point.y - self.VIEWFRAME_CENTER_Y
        center_x_dist_to_centroid = point.x - self.VIEWFRAME_CENTER_X
        self.distance_centroid_to_center = math.sqrt(center_y_dist_to_centroid * center_y_dist_to_centroid\
                                                    + center_x_dist_to_centroid * center_x_dist_to_centroid)

        self.centroid_delta_y_pub.publish(center_y_dist_to_centroid)
        self.centroid_delta_x_pub.publish(center_x_dist_to_centroid)

        if (self.distance_centroid_to_center < self.VIEWFRAME_CENTROID_RADIAL_THRESHOLD_TO_CENTER):
            print('Centroid distance from center: {}'.format(self.distance_centroid_to_center))
            self.stable_counts_centroid += 1
        else:
            self.stable_counts_centroid = 0

    def distance_cb(self, distance):
        if(distance.data < self.BUOY_DISTANCE_THRESHOLD)   :
            print('Distance from buoy: {}'.format(self.distance))
            self.stable_counts_distance += 1
        else:
            self.stable_counts_distance = 0
    

    def execute(self, userdata):
        self.navigate_to_buoy_heave_pid_setpoint_pub.publish(0)
        self.navigate_to_buoy_sway_pid_setpoint_pub.publish(0)
        self.distance_to_buoy_pid_setpoint_pub.publish(self.BUOY_TARGET_DISTANCE)

        self.navigate_to_buoy_sway_pid_enable_pub.publish(True)
        self.navigate_to_buoy_heave_pid_enable_pub.publish(True)
        rospy.loginfo('Executing state SwimStraight')
        '''
        1) Align with centroid plus stabilize
        2) If aligned, if we're not stable at threshold distance, surge forward - enable PID
        2.5)           if stable at threshold distance, exit loop
        3) If misaligned, disable PID and back to step 1
        
        while not (self.stable_counts_centroid >= self.COUNTS_FOR_STABILITY):
            remaining_counts = self.COUNTS_FOR_STABILITY - self.stable_counts_centroid
            rospy.loginfo_throttle(1, 'Moving towards centroid: Need {} more stable readings'.format(remaining_counts))
            self.distance_to_buoy_pid_setpoint_pub.publish(THRESHOLD_DISTANCE)
            self.distance_to_buoy_pid_enable_pub.publish(True)
            while not (self.stable_counts_distance >= self.COUNTS_FOR_STABILITY):
                remaining_counts = self.COUNTS_FOR_STABILITY - self.stable_counts_distance
                rospy.loginfo_throttle(1, 'Moving towards buoy: Need {} more stable readings'.format(remaining_counts))
                '''
        while(True):
            
            if(self.stable_counts_centroid >= self.COUNTS_FOR_STABILITY):
                if(self.stable_counts_distance >= self.COUNTS_FOR_STABILITY):
                    self.navigate_to_buoy_heave_pid_enable_pub.publish(False)
                    self.navigate_to_buoy_sway_pid_enable_pub.publish(False)
                    self.distance_to_buoy_pid_enable_pub.publish(False)
                    return 'ReachedBuoy'
                else:
                    remaining_counts = self.COUNTS_FOR_STABILITY - self.stable_counts_distance
                    rospy.loginfo_throttle(1, 'Moving towards buoy: Need {} more stable readings'.format(remaining_counts))
                    self.distance_to_buoy_pid_enable_pub.publish(True)

            else:
                remaining_counts = self.COUNTS_FOR_STABILITY - self.stable_counts_centroid
                rospy.loginfo_throttle(1, 'Moving towards centroid: Need {} more stable readings'.format(remaining_counts))
                self.distance_to_buoy_pid_enable_pub.publish(False)