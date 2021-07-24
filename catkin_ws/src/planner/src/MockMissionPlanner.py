import rospy
import roslaunch
import smach
import math
import actionlib
import time
from cv.msg import CvTarget
from std_msgs.msg import Bool, Float64, Float32MultiArray
from blinky.msg import TaskStatus
from geometry_msgs.msg import Vector3Stamped, Point
from threading import Thread

from planner.msg import LaneDetectorCenteringAction, LaneDetectorCenteringGoal,  LaneDetectorAlignmentAction, LaneDetectorAlignmentGoal

# Navigation Target coordinates 
# X     : DVL
# Y     : DVL
# z     : Depth Senesor
# roll  : IMU
# Yaw   : IMU
# Pitch : IMU

# Global variables
#Global COUNTS_FOR_STABILITY = 2


class GateState(smach.State):
    # The gate state : ded reckoning
    # 0) Read the depth 
    # 1) Submerge to a known depth
    # 2) Check for stability at this depth
    # 3) surge forward for an empircally determined amount of time. (yikes.)
    # 4) Transitions to the next state

    def __init__(self):

        # Initialize class as state and define transitions
        smach.State.__init__(self, outcomes=['gatePassed', 'gateMissed'])

        #Define parameters and variables
        self.DEPTH_SETPOINT      = 2.5 # meters
        self.DEPTH_THRESHOLD     = 0.2 # meters
        # self.STABLE_COUNTS       = 1 TODO remove?
        self.SURGE_MAGNITUDE     = 1   # one day I'll know what these units are. Start small and increment in pool testing.
        self.SURGE_DURATION      = 10  # seconds

        self.current_count       = 0   # the counter variable to determine if we are stable at the setpoint.
        self.stable_at_depth     = False
        self.depth_achieved_time = None 
        self.done_surging        = False

        #Defining the subscriber to read the current Depth
        self.depth_sub = rospy.Subscriber('/state_estimation/depth', Float64, self.depth_cb)

        #Define the publishers to set depth setpoint and publish to surge
        self.depth_enable_pub      = rospy.Publisher('/controls/depth_pid/pid_enable' , Bool    , queue_size=1)
        self.depth_setpoint_pub    = rospy.Publisher('/controls/depth_pid/setpoint'   , Float64 , queue_size=1)
        self.surge_magnitude_pub   = rospy.Publisher('/controls/superimposer/surge'   , Float64 , queue_size=1)

        # Define the publisher for blinky, publish a task...just a random one for now
        #self.blinky_pub            = rospy.Publisher('/task'   , TaskStatus , queue_size=1)
        #self.taskmsg        = TaskStatus()
        #self.taskmsg.action = 1
        #self.taskmsg.task   = 1
        #self.blinky_pub.publish (taskmsg)
        
        #Turn on the Depth PID
        self.depth_setpoint_pub.publish(self.DEPTH_SETPOINT)
        self.depth_enable_pub.publish(True) 

    def depth_cb(self, msg):
        self.depth = msg.data # takes the depth float 64 from the subscriber and sets it to a variable
        print('inside callback. Current depth: {} '.format(self.depth))
        # Check for stability at depth
        if abs(self.depth-self.DEPTH_SETPOINT) < self.DEPTH_THRESHOLD :
            #print('recieved a stable count')
            self.current_count += 1
        else:
            self.current_count   = 0

        if self.current_count > self.STABLE_COUNTS : # We are at depth! 
            self.stable_at_depth = True   # I am assuming this never needs to be set false. Danger danger. 

    def execute(self, userdata):
        rospy.loginfo('Inside Gate State')
        # Check if we are stable at the setpoint? 
        while ( not self.done_surging): # Loop inside here untill we are ready to move to the next state
            #print out the number of counts we are waiting for
            self.remaining_counts = self.STABLE_COUNTS - self.current_count
            if self.remaining_counts > 0 :
                rospy.loginfo_throttle(1, 'Attaining Depth: Need {} more stable readings'.format(self.remaining_counts))      

            if self.stable_at_depth == True:
                #Get the timestamp of the first moment the robot is at depth
                if self.depth_achieved_time is None: 
                    self.depth_achieved_time = rospy.get_time()
                # Surge for a known duration, then exit the state
                self.time_remaining = self.SURGE_DURATION - (rospy.get_time() - self.depth_achieved_time)
                if ( self.time_remaining >0):
                    self.surge_magnitude_pub.publish(self.SURGE_MAGNITUDE)
                    rospy.loginfo_throttle(1, 'Surging: Time left is {} seconds'.format(self.time_remaining))
                else:
                    self.surge_magnitude_pub.publish(0)
                    self.done_surging = True
                    return 'gatePassed'


class GridSearch(smach.State):
    def __init__(self):
        # This Executes a raster scan to search for something, probably a lane
        # 1) Turn 90 degrees using the IMU 
        # 2) Surge for half the predetermined time (long)
        # 3) Turn 180 degrees using the IMU
        # 4) Surge for a predetermined time (long)

        # loop until we find lane or reach max_value
        # 5) Turn 90 degrees using the IMU
        # 6) Surge for a predetermined time (short)
        # 7) Turn 90 degrees using the IMU
        # 8) Surge for a predetermined time (long), call lane detector after set time
        # 9) Change sign of 90 degrees turn
        
        smach.State.__init__(self, outcomes=['missionSucceeded'])
        self.SHORT_TIME               = 1
        self.NUMBER_OF_SHORT_PER_LONG = 5 
        self.MAX_ITERATION            = 5
        self.YAW_THRESHOLD            = 0.1 * 3.14 / 2
        self.SURGE_MAGNITUDE          = 1
        self.STABLE_COUNTS            = 1

        self.turn_angle          = 90
        self.current_yaw         = None 
        self.current_yaw         = 0  
        self.stable_at_yaw       = False   
        self.done_surging        = False   
        self.current_count       = 0
        self.angle_achieved_time = None 

        self.current_pose_sub      = rospy.Subscriber('/robot_state', Vector3Stamped, self.IMU_cb)
        self.yaw_enable_pub        = rospy.Publisher('/controls/yaw_pid/pid_enable' , Bool    , queue_size=1)
        self.yaw_setpoint_pub      = rospy.Publisher('/controls/yaw_pid/setpoint'   , Float64 , queue_size=1)
        self.surge_magnitude_pub   = rospy.Publisher('/controls/superimposer/surge'   , Float64 , queue_size=1)


    def IMU_cb(self, msg):
        self.current_yaw = msg.vector.z

    def turn_90_degrees(self, direction):
        print("entered turn 90 deg")
        if(direction == 'left'):
            print("goign left")
            target_yaw = self.current_yaw - 3.14/2.0
        elif(direction=='right'):
            print("going right")
            target_yaw = self.current_yaw + 3.14/2.0
        else:
            print("Invalid direction. Choose 'left' or 'right'.")
            return

        self.yaw_setpoint_pub.publish(target_yaw)
        self.yaw_enable_pub.publish(True)
        self.current_count   = 0
        while not self.stable_at_yaw:
            print("inside 90 deg while loop")
            print("target_yaw {}".format(target_yaw))
            if abs(self.current_yaw-target_yaw) < self.YAW_THRESHOLD:
                print('recieved a stable count')
                self.current_count += 1
            else:
                self.current_count = 0

            if self.current_count > self.STABLE_COUNTS : # We are at correct yaw angle! 
                print("jolly good")
                self.stable_at_yaw = True   # I am assuming this never needs to be set false. Danger danger. 

    def move_forward(self):
         while ( not self.done_surging): # Loop inside here untill we are ready to move to the next state
            #print out the number of counts we are waiting for
            self.remaining_counts = self.STABLE_COUNTS - self.current_count
            if self.remaining_counts > 0 :
                rospy.loginfo_throttle(1, 'Attaining yaw angle: Need {} more stable readings'.format(self.remaining_counts))      

            if self.stable_at_yaw == True:
                #Get the timestamp of the first moment the robot is at depth
                if self.angle_achieved_time is None: 
                    self.angle_achieved_time = rospy.get_time()
                # Surge for a known duration, then exit the state
                self.time_remaining = self.SHORT_TIME - (rospy.get_time() - self.angle_achieved_time)
                if ( self.time_remaining >0):
                    self.surge_magnitude_pub.publish(self.SURGE_MAGNITUDE)
                    rospy.loginfo_throttle(1, 'Surging: Time left is {} seconds'.format(self.time_remaining))
                else:
                    self.surge_magnitude_pub.publish(0)
                    self.done_surging = True
                    return
        

    def execute(self, userdata):
        yaw_before_turn = self.current_yaw
        self.turn_90_degrees('left')
    
        self.move_forward()
        return 'missionSucceeded'
    

        if seeingLane: # !!! this condition is not defined
            return 'pointingToNextTask'
        
        else:
            return 'notSeeingLane'


class LaneDetector(smach.State):

    def __init__(self):
        # 0) Assume we see a little bit of the lanes when we enter the state
        # 1) Move towards the centroid of the lane(s) detetected.
        #    If we do not see the entirety of the lanes, the centroid will be self-correcting and 
        #    we will eventually reach the ultimate stable centroid of the 2 lanes.
        # 2) Derive a new heading from the lane detector (new heading should be towards the next task)
        # 3) Set the Yaw PID setpoint to 0 degrees with respect to the lane/new heading
        # 4) Enable the Yaw PID and wait until we reach a stable state
        # 5) Move on to next smach state which will be to surge forward
    
        # Initialize class as state and define transitions
        smach.State.__init__(self, outcomes=['AlignmentSuccess'])

        # Global constants (maybe this should be a constant even outside of the class)
        self.COUNTS_FOR_STABILITY                           = 30 # This can still change, we are testing...

        # Centering constants
        self.VIEWFRAME_PIXEL_WIDTH                          = 720 # testing # pixels
        self.VIEWFRAME_PIXEL_HEIGHT                         = 420 # testing # pixels
        self.VIEWFRAME_CENTER_X                             = self.VIEWFRAME_PIXEL_WIDTH / 2.0
        self.VIEWFRAME_CENTER_Y                             = self.VIEWFRAME_PIXEL_HEIGHT / 2.0
        self.IMAGE_CENTER_POINT                             = Point(x = self.VIEWFRAME_CENTER_X, y = self.VIEWFRAME_CENTER_Y)
        self.VIEWFRAME_CENTROID_RADIAL_THRESHOLD_TO_CENTER  = 0.3 * self.VIEWFRAME_PIXEL_HEIGHT # pixels

        # Alignment constants
        self.TARGET_ANGLE                                   = 0.0
        self.YAW_ALIGNMENT_THRESHOLD_TO_NEXT_TASK           = 10 * 3.14 / 180


    def execute(self, userdata):

        # rospy.loginfo('Executing state LaneDetector')

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

    
class NavigateToBuoyTask(smach.State):
    def __init__(self):
        # 0) Assume we are aligned with the lane right below us. The front camera should be able to see the buoy since it is white and big.
        # 1) Center the Buoy in the field of view using a PID on the convex hull
        # 2) Use the CV detector to report a probability that the current image in frame is the one we want at regular intervals
        # 3) If the probability exceeds a threshold, begin surging
        # END OF TASK
        # 4) Continue surging untill the convex hull takes up the whole field of vision, then continue for a hardcoded time (test!) to actually touch it
        # 5) Transition into the next task

        self.VIEWFRAME_PIXEL_WIDTH             = 720 #TODO
        self.VIEWFRAME_PIXEL_HEIGHT            = 420 #TODO
        self.VIEWFRAME_CENTER_X                = self.VIEWFRAME_PIXEL_WIDTH / 2.0
        self.VIEWFRAME_CENTER_Y                = self.VIEWFRAME_PIXEL_HEIGHT / 2.0
        self.VIEWFRAME_CENTROID_RADIAL_THRESHOLD_TO_CENTER = 0.05 * self.VIEWFRAME_PIXEL_HEIGHT # in pixels
        self.BUOY_TARGET_DISTANCE = 1 # in meters(?)
        self.BUOY_DISTANCE_THRESHOLD = 0.15
        self.COUNTS_FOR_STABILITY = 2


        smach.State.__init__(self, outcomes=['BuoyReached'])

        self.navigate_to_buoy_sway_pid_enable_pub = rospy.Publisher('/buoy_sway_pid/enable', Bool, queue_size = 1)
        self.navigate_to_buoy_sway_pid_setpoint_pub = rospy.Publisher('/buoy_sway_pid/setpoint', Float64, queue_size = 1)
        self.navigate_to_buoy_heave_pid_enable_pub = rospy.Publisher('/buoy_heave_pid/enable', Bool, queue_size = 1)
        self.navigate_to_buoy_heave_pid_setpoint_pub = rospy.Publisher('/buoy_heave_pid/setpoint', Float64, queue_size = 1)
        self.centroid_delta_y_pub = rospy.Publisher('/navigate_to_buoy/centroid_delta_y', Float64, queue_size = 1)
        self.centroid_delta_x_pub = rospy.Publisher('/navigate_to_buoy/centroid_delta_x', Float64, queue_size = 1)
        self.distance_to_buoy_pid_enable_pub = rospy.Publisher('/buoy_distance_pid/enable', Bool, queue_size = 1)
        self.distance_to_buoy_pid_setpoint_pub = rospy.Publisher('/buoy_distance_pid/setpoint', Float64, queue_size = 1)

        self.distance_to_buoy_sub = rospy.Subscriber('/cv/buoy_distance', Float64, self.distance_cb)


        self.target_sub = rospy.Subscriber('/cv/front_cam_target', Point, self.target_cb)

        self.current_stable_counts_centroid = 0
        self.current_stable_counts_distance = 0

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
            self.current_stable_counts_centroid += 1
        else:
            self.current_stable_counts_centroid = 0

    def distance_cb(self, distance):
        if(distance.data < self.BUOY_DISTANCE_THRESHOLD)   :
            print('Distance from buoy: {}'.format(self.distance))
            self.current_stable_counts_distance += 1
        else:
            self.current_stable_counts_distance = 0
    

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
        
        while not (self.current_stable_counts_centroid >= self.COUNTS_FOR_STABILITY):
            remaining_counts = self.COUNTS_FOR_STABILITY - self.current_stable_counts_centroid
            rospy.loginfo_throttle(1, 'Moving towards centroid: Need {} more stable readings'.format(remaining_counts))
            self.distance_to_buoy_pid_setpoint_pub.publish(THRESHOLD_DISTANCE)
            self.distance_to_buoy_pid_enable_pub.publish(True)
            while not (self.current_stable_counts_distance >= self.COUNTS_FOR_STABILITY):
                remaining_counts = self.COUNTS_FOR_STABILITY - self.current_stable_counts_distance
                rospy.loginfo_throttle(1, 'Moving towards buoy: Need {} more stable readings'.format(remaining_counts))
                '''
        while(True):
            
            if(self.current_stable_counts_centroid >= self.COUNTS_FOR_STABILITY):
                if(self.current_stable_counts_distance >= self.COUNTS_FOR_STABILITY):
                    self.navigate_to_buoy_heave_pid_enable_pub.publish(False)
                    self.navigate_to_buoy_sway_pid_enable_pub.publish(False)
                    self.distance_to_buoy_pid_enable_pub.publish(False)
                    return 'ReachedBuoy'
                else:
                    remaining_counts = self.COUNTS_FOR_STABILITY - self.current_stable_counts_distance
                    rospy.loginfo_throttle(1, 'Moving towards buoy: Need {} more stable readings'.format(remaining_counts))
                    self.distance_to_buoy_pid_enable_pub.publish(True)

            else:
                remaining_counts = self.COUNTS_FOR_STABILITY - self.current_stable_counts_centroid
                rospy.loginfo_throttle(1, 'Moving towards centroid: Need {} more stable readings'.format(remaining_counts))
                self.distance_to_buoy_pid_enable_pub.publish(False)


class TouchBuoyTask(smach.State):
    '''
    This state assumes that we have already navigated to the BUOY, and it is centered 
    in the viewframe. 
    The 'lateral' (heave and sway) PIDs from the previous state will still be running, such that the
    buoy will be maintined in the center of the viewframe. 

    This state is responsible for
        1. launching the KNN detector from the cv package
        2. Listening to the output topics of this package to await image detections
        3. Upon recognition of the target, surging to touch it
        4. Transitioning into the next task ('navigate to the surface') 
    
    This state is making some...potentially nasty assumptions. THe one that I am worried about is
       1. If the "wrong" image is on the BUOY, we will need to wait untill the buoy rotates. 
          Its not clear to me that just leaving the PIDS on will robustly handle this . We'll see in pool testing 
    
    The syntax for launching the launch file comes from
    http://wiki.ros.org/roslaunch/API%20Usage 

    '''
    def __init__(self):
        #Define the smach transitions that are possible
        smach.State.__init__(self, outcomes=['TouchedTheBuoy'])

        self.SURGE_TIME       = 10    # Units of seconds
        self.SURGE_STRENGTH   = 1     # A small number, to be changed during pool testing
        self.initial_time     = None  # To be overwritten when the target is recognized for the first time
        self.target_acquired  = False # To know if we need to start the timer!
        self.finished_surging = False

        self.knn_sub               = rospy.Subscriber('/objects', Float32MultiArray, self.knn_cb)
        self.surge_magnitude_pub   = rospy.Publisher('/controls/superimposer/surge'   , Float64 , queue_size=1)
        
    def knn_cb(self):
        '''
        WE shouuuuuuld already have the buoy in the center of the viewframe due to the
        Colour thresholding PIDs. all we need to do is surge.
        As of now, this does NOT wait for several consecutive counts, and surges immediately 
        upon recognizing a target.
        '''
        if not self.target_aquired: 
            self.initial_time = time.time()
            self.surge_toward_target()
            self.target_aquired = True
        if self.target_acuired == True:
            # The surge function hangs, and does all the surging! 
            # If we're already surging, we just want this to do nothing!
            return

    def surge_toward_target(self):
        while time.time()- self.initial_time < self.SURGE_TIME : 
            self.surge_magnitude_pub(self.SURGE_STRENGTH)
        self.finished_surging = True
        return


    def execute(self, userdata):
        # Launch the knn Detector Launch file
        # This wild magic is from the link in the comment above

        rospy.init_node('knn_launcher_en_Mapping', anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        knn_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/haier/catkin_ws/src/testapi/launch/test_node.launch"])
        knn_launch.start()
        rospy.loginfo("knn launcher started")
        
        '''
        package = 'cv'
        executable = 'knnDetector'
        node = roslaunch.core.Node(package, executable)

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        knnDetector_process = launch.launch(node)
        print(knnDetector_process.is_alive())
        '''
        #Wait untill surging is finished
        while not self.finished_surging:
            #do nothing! Wait for the KNN detector to publish something on the /objects topic!
            pass
        
        #knnDetector_process.stop()
        knn_launch.shutdown()
        return 'TouchedTheBuoy'
        

class NavitageToSurfacingTask(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['missionSucceeded'])

        #Defining the subscriber to read the current Hydrophones Heading
        self.hydrophones_sub = rospy.Subscriber('/hydrophones/heading', Float64, self.hydrophones_cb) 
        
        #TODO Stub values, Needs pool testing
        self.YAW_THRESHOLD      = Float64()
        self.YAW_THRESHOLD.data = 20 * 3.14/180   # 20 degrees, but converted to Radians, threshold for being "aligned" to the pinger
        self.ARRIVAL_THRESHOLD  = Float64()       #  Threshold for checking if the pinger is "behind" us and we have arrived 
        self.ARRIVAL_THRESHOLD.data = 50 * 3.14/180       
        self.STABLE_COUNT       = 2             # This defines the number of measurements we need to take within the yaw threshold to be stable
        self.alignment_count    = 0
        self.arrival_count      = 0
        self.ever_aligned       = False         # A check if we have ever been aligned to the pinger.
        self.YAW_TARGET         = Float64()          # This will depend on how the hydrophones are mounted, I am guessing 0 for now. 
        self.YAW_TARGET.data    = 0
        self.SURGE_MAGNITUDE    = 1             # I have no idea what the units are, be careful.
        self.successful_surface = False

        #Set up all of the publishers I will use later
        self.yaw_enable_pub      = rospy.Publisher('/controls/yaw_pid/pid_enable' , Bool    , queue_size=1)
        self.yaw_setpoint_pub    = rospy.Publisher('/controls/yaw_pid/setpoint'   , Float64 , queue_size=1)
        
        self.surge_magnitude_pub = rospy.Publisher('/controls/superimposer/surge'   , Float64 , queue_size=1)
        self.depth_enable_pub    = rospy.Publisher('/controls/depth_pid/pid_enable' , Float64 , queue_size=1)
        
        #Enable the yaw PID, try to align to the pinger.
        self.yaw_setpoint_pub.publish(self.YAW_TARGET)  
        self.yaw_enable_pub.publish(True)

        # Define the publisher for blinky, publish a task...just a random one for now
        
        #self.blinky_pub            = rospy.Publisher('/task'   , TaskStatus , queue_size=1)
        #self.taskmsg        = TaskStatus()
        #self.taskmsg.action = 2
        #self.taskmsg.task   = 2
        #self.blinky_pub.publish (taskmsg)

    def hydrophones_cb(self, msg):
        self.heading = msg # takes the heading float 64 from the subscriber and sets it to a variable
        # Check if we are stable! If we are outside the threshold, restart the counter.
        #------------------------------------- Checking for alignment to the pinger -------------------------------------
        if abs(self.heading.data - self.YAW_TARGET.data) < self.YAW_THRESHOLD.data :
            self.alignment_count += 1
        else:
            self.alignment_count  = 0

        if self.alignment_count > self.STABLE_COUNT : # We are aligned, go forward
            self.surge_magnitude_pub.publish(self.SURGE_MAGNITUDE) 
            rospy.loginfo('Aligned. Turning on the Thrusters')
            self.ever_aligned = True
        else :
            self.surge_magnitude_pub.publish(0) # We are very unaligned. Stop and re-align.
            rospy.loginfo('Misaligned. Turning off the Thrusters')

        #------------------------------------- Checking for arrival at the pinger -------------------------------------
        # If ever aligned and now outside the arrival threshold?
        print("everaligned {}  Arrival Threshold {}".format(self.ever_aligned, abs(self.heading.data)))
        if (self.ever_aligned and (abs(self.heading.data) > self.ARRIVAL_THRESHOLD.data)):
            print("Inside the if statement")
            # Check if this is stable
            self.surge_magnitude_pub.publish(0) #First, turn off thrusters
            self.arrival_count +=1
            rospy.loginfo('Arrived?. Waiting for {} more stable counts'.format(self.STABLE_COUNT-self.arrival_count))
            if self.arrival_count > self.STABLE_COUNT :
                #If it is stable, surface!
                rospy.loginfo('Arrived, surfacing')
                self.depth_enable_pub.publish(0)
                self.successful_surface = True

       

    def execute(self, userdata):
        #Use the Hydrophones to align the AUV to the pinger
           #1) Read the hydrophone heading - DONE
           #2) Set the YAW PID setpoint to 0 (pointing toward the pinger) and Enable the yaw PID - DONE
           #3) Check if we are stable at the setpoint within some tolerance - DONE

        # Surge toward the pinger, while checking if it is still in front of us
            #1) Set surge topic to some small number to move toward the pinger - DONE
            #2) CHeck the error signal of the Yaw PID. 
               # IF there is a spike beyond an emprical threshold, think the pinger is behind us. STOP, and see if it's stable!
               # IF it is stable, surface !

        # Turn the Depth PID off to surface
        rospy.loginfo('Executing state Navigate to surfacing task')
        while not self.successful_surface:
            rospy.loginfo_throttle(1 ,'Alignment count : {} | arrival count: {}'.format(self.alignment_count , self.arrival_count) )


        if self.successful_surface: # !!! this condition is not defined
            return 'missionSucceeded'
        else:
            pass

# main
def main():
    rospy.init_node('mockMissionPlanner', anonymous=True)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['missionFailed', 'missionSucceeded'])

    # Open the container
    with sm:
        # Add states to the container
        # This is where the  transitions are defined!

        # Overall plan for state transitions
        # 1) Gate State - done
        # 2) Grid Search - done
        # 3) Lane Detector - done
        # 4) Surge To Buoy
        # 5) Buoy Task
        # 6) Grid Search - done
        # 7) Lane Detector - done
        # 8) Surge To Garlic
        # 9) Garlic Task
        # 10) Pinger To Torpedo
        # 11) Torpedo Task
        # 12) Pinger To Surface
        # 13) JOLLY GOOD
        
        '''
        Gate task. Transitions Directly to pinger task...for now.
        smach.StateMachine.add('GateState', GateState(), 
                                transitions={'gatePassed':'NavitageToSurfacingTask',
                                            'gateMissed':'missionFailed'})

        smach.StateMachine.add('GridSearch', GridSearch(), 
                                transitions={'missionSucceeded':'missionSucceeded'})

        The surface task. Finishes up the mission.
        smach.StateMachine.add('NavitageToSurfacingTask', NavitageToSurfacingTask(), 
                                transitions={'missionSucceeded':'missionSucceeded'})
        '''
        smach.StateMachine.add('TouchBuoyTask',TouchBuoyTask(),
                                transitions={'TouchedTheBuoy':'missionSucceeded'})
                                #In competition this will transision into NavigateToSurfacingTask

        '''
        smach.StateMachine.add('LaneDetectorForBuoy',LaneDetector(),
                                transitions={'AlignmentSuccess':'NavigateToBuoyTask'})

        smach.StateMachine.add('NavigateToBuoyTask',NavigateToBuoyTask(),
                                transitions={'BuoyReached':'missionSucceeded'})
        '''

    # In order to get smach to respond to ctrl+c we run it in a different
    # thread and request a preempt on ctrl+c.
    smach_thread = Thread(target=lambda: sm.execute())
    smach_thread.start()

    # It is necessary to use the on_shutdown method to request the preempt
    # rather than waiting until after rospy spin to do so. Otherwise, the
    # state machine will not respond to ctrl+c.
    rospy.on_shutdown(sm.request_preempt)
    rospy.spin()
    smach_thread.join()

    # Execute SMACH plan
    #outcome = sm.execute()


if __name__ == '__main__':
    main()