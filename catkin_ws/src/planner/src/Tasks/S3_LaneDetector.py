import rospy
import smach
import actionlib

from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Point
from cv.msg import CvTarget
from planner.msg import LaneDetectorCenteringAction, LaneDetectorCenteringGoal,  LaneDetectorAlignmentAction, LaneDetectorAlignmentGoal

class LaneDetector(smach.State):
    # 0) Assume we see a little bit of the lanes when we enter the state
    # 1) Move towards the centroid of the lane(s) detetected.
    #    If we do not see the entirety of the lanes, the centroid will be self-correcting and 
    #    we will eventually reach the ultimate stable centroid of the 2 lanes.
    # 2) Derive a new heading from the lane detector (new heading should be towards the next task)
    # 3) Set the Yaw PID setpoint to 0 degrees with respect to the lane/new heading
    # 4) Enable the Yaw PID and wait until we reach a stable state
    # 5) Move on to next smach state which will be to surge forward

    def __init__(self):
    
        # Initialize class as state and define transitions
        smach.State.__init__(self, outcomes=['AlignmentSuccess'])

        # Abstract constants
        self.COUNTS_FOR_STABILITY                           = 30 # This can still change, we are testing...

        # Centering constants
        self.VIEWFRAME_PIXEL_WIDTH                          = 720 # TODO testing # pixels
        self.VIEWFRAME_PIXEL_HEIGHT                         = 420 # TODO testing # pixels
        self.VIEWFRAME_CENTER_X                             = self.VIEWFRAME_PIXEL_WIDTH / 2.0
        self.VIEWFRAME_CENTER_Y                             = self.VIEWFRAME_PIXEL_HEIGHT / 2.0
        self.IMAGE_CENTER_POINT                             = Point(x = self.VIEWFRAME_CENTER_X, y = self.VIEWFRAME_CENTER_Y)
        self.VIEWFRAME_CENTROID_RADIAL_THRESHOLD_TO_CENTER  = 0.3 * self.VIEWFRAME_PIXEL_HEIGHT # pixels

        # Alignment constants
        self.TARGET_ANGLE                                   = 0.0
        self.YAW_ALIGNMENT_THRESHOLD_TO_NEXT_TASK           = 10 * 3.14 / 180


    def execute(self, userdata):

        # rospy.loginfo('Executing state LaneDetector')

        '''
        I find there to be a lot of print statement in the following function and it feels a little crowded,
        but I don't really know how to improve this situation...
        '''

        # Centering ActionServer
        print("Starting centering client")
        print("Creating client...")
        client = actionlib.SimpleActionClient('LDCentering', LaneDetectorCenteringAction)
        print("Client created, waiting for server...")
        client.wait_for_server()
        print("Found server, creating goal...")
        goal = LaneDetectorCenteringGoal(image_center_point = self.IMAGE_CENTER_POINT)
        print("Goal created, sending goal...")
        print(goal)
        client.send_goal(goal)
        print("Goal sent, waiting for result...")
        client.wait_for_result()
        print("Result received")
        print(client.get_result())
        print("Stop centering ActionServer")

        # Alignment ActionServer
        print("Starting alignment client")
        print("Creating client...")
        client = actionlib.SimpleActionClient('LDAlignment', LaneDetectorAlignmentAction)
        print("Client created, waiting for server...")
        client.wait_for_server()
        print("Found server, creating goal...")
        goal = LaneDetectorAlignmentGoal(image_angle_target = Float64(data= self.TARGET_ANGLE))
        print("Goal created, sending goal...")
        print(goal)
        client.send_goal(goal)
        print("Goal sent, waiting for result...")
        client.wait_for_result()
        print("Result received")
        print(client.get_result())
        print("Stop centering ActionServer")

        # We should probably add timeout and logic handling "failure"    

        return 'AlignmentSuccess'