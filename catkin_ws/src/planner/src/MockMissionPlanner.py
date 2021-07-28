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

from Tasks.GateTask     import GateTask
from Tasks.GridSearch   import GridSearch
from Tasks.LaneDetector import LaneDetector

# Navigation Target coordinates 
# X     : DVL
# Y     : DVL
# z     : Depth Senesor
# roll  : IMU
# Yaw   : IMU
# Pitch : IMU

    
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

        self.SURGE_TIME       = 2    # Units of seconds
        self.SURGE_STRENGTH   = 2     # A small number, to be changed during pool testing
        self.initial_time     = None  # To be overwritten when the target is recognized for the first time
        self.target_acquired  = False # To know if we need to start the timer!
        self.finished_surging = False

        self.knn_sub               = rospy.Subscriber('/objects', Float32MultiArray, self.knn_cb)
        self.surge_magnitude_pub   = rospy.Publisher('/controls/superimposer/surge'   , Float64 , queue_size=1)
        
    def knn_cb(self, objectArray):
        '''
        WE shouuuuuuld already have the buoy in the center of the viewframe due to the
        Colour thresholding PIDs. all we need to do is surge.
        As of now, this does NOT wait for several consecutive counts, and surges immediately 
        upon recognizing a target.
        '''
        print('Data on topic : {}'.format(objectArray))
        #Only do something if the data on the topic isn't empty
        if len(objectArray.data)> 0:
            if not self.target_acquired: 
                self.initial_time = time.time()
                self.surge_toward_target()
                self.target_acquired = True
            if self.target_acquired == True:
                # The surge function hangs, and does all the surging! 
                # If we're already surging, we just want this to do nothing!
                return

    def surge_toward_target(self):
        while time.time()- self.initial_time < self.SURGE_TIME : 
            self.surge_magnitude_pub.publish(self.SURGE_STRENGTH)
            rospy.sleep(0.1)
        self.finished_surging = True
        return


    def execute(self, userdata):
        # Launch the knn Detector Launch file
        # This wild magic is from the link in the comment above

        #rospy.init_node('en_Mapping', anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        knn_launch = roslaunch.parent.ROSLaunchParent(uuid, 
                    ["/home/tommy/robotics/AUV-2020/catkin_ws/src/cv/launch/knnDetector.launch"])
        knn_launch.start()
        rospy.loginfo("knn launcher started")
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


    '''
    # This threading syntax is from the old repository, and I'm not sure 
    # Where it came from originally. The threading is causing an error when trying to 
    # Call launch files, I'm removing it for now.

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
    '''
    
    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()