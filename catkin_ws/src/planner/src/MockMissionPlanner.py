import rospy
import smach
from std_msgs.msg import Bool, Float64
from blinky.msg import TaskStatus

# Navigation Target coordinates 
# X     : DVL
# Y     : DVL
# z     : Depth Senesor
# roll  : IMU
# Yaw   : IMU
# Pitch : IMU

#
# define state GateState


class GateState(smach.State):
    '''
    The gate state : ded reckoning
    0) Read the depth 
    1) Submerge to a known depth
    2) Check for stability at this depth
    3) surge forward for an empircally determined amount of time. (yikes.)
    4) Transitions to the next state

    '''
    def __init__(self):
        #Define the state transitions!
        smach.State.__init__(self, outcomes=['gatePassed', 'gateMissed'])

        #Define parameters and variables
        self.DEPTH_SETPOINT      = 2.5 # units of meters
        self.DEPTH_THRESHOLD     = 0.2 # meters
        self.STABLE_COUNTS       = 1
        self.SURGE_MAGNITUDE     = 1   # one day I'll know what these units are. Start small and increment in pool testing.
        self.SURGE_DURATION      = 10  # units of seconds

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
        # 1) Surge for a predetermined time (long)
        # 2) Turn 90 degrees using the IMU
        # 3) S
        smach.State.__init__(self, outcomes=['pointingToNextTask', 'notSeeingLane'])

    def execute(self, userdata):
        #
        rospy.loginfo('Executing state LaneDetector')

        if seeingLane: # !!! this condition is not defined
            return 'pointingToNextTask'
        
        else:
            return 'notSeeingLane'


# define state LaneDetector
class LaneDetector(smach.State):
    def __init__(self):
        # 1) Center the lane in the viewframe
        # 2) derive a new heading from the lane detector
        # 3) Set the Yaw PID setpoint to 
        smach.State.__init__(self, outcomes=['pointingToNextTask', 'notSeeingLane'])

    def execute(self, userdata):
        #
        rospy.loginfo('Executing state LaneDetector')

        if seeingLane: # !!! this condition is not defined
            return 'pointingToNextTask'
        
        else:
            return 'notSeeingLane'

class BuoyTask(smach.State):
    def __init__(self):
        # 1) Center the Buoy in the field of view using a PID on the convex hull
        # 2) Use the CV detector to report a probability that the current image in frame is the one we want at regular intervals
        # 3) If the probability exceeds a threshold, begin surging
        # 4) Continue surging untill the convex hull takes up the whole field of vision, then continue for a hardcoded time (test!) to actually touch it
        # 5) Transition into the next task

        smach.State.__init__(self, outcomes=['atNextTask', 'notAtNextTask'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SwimStraight')
        if atTask: # !!! this condition is not defined
            return 'pointingToNextTask'
        
        else:
            return 'notSeeingLane'



class NavitageToSurfacingTask(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['missionSuceeded'])

        #Defining the subscriber to read the current Hydrophones Heading
        self.hydrophones_sub = rospy.Subscriber('/hydrophones/heading', Float64, self.hydrophones_cb) 
        
        #TODO Stub values, Needs pool testing
        self.YAW_THRESHOLD      = Float64()
        self.YAW_THRESHOLD.data = 20 * 3.14/180   # 20 degrees, but converted to Radians, threshold for being "aligned" to the pinger
        self.ARRIVAL_THRESHOLD  = Float64()       #  Threshold for checking if the pinger is "behind" us and we have arrived 
        self.ARRIVAL_THRESHOLD.data = 50 * 3.14/180       
        self.STABLE_COUNT       = 5             # This defines the number of measurements we need to take within the yaw threshold to be stable
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
        if (self.ever_aligned and (self.heading.data > self.ARRIVAL_THRESHOLD)):
            # Check if this is stable
            self.surge_magnitude_pub.publish(0) #First, turn off thrusters
            self.arrival_count +=1
            rospy.loginfo('Arrived?. Waiting for {} more stable counts'.format(self.STABLE_COUNT-self.arrival_count))
            if arrival_count > STABLE_COUNT :
                #If it is stable, surface!
                rospy.loginfo('Arrived, surfacing')
                self.depth_enable_pub(0)
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


        if successful_surface: # !!! this condition is not defined
            return 'missionSuceeded'
        else:
            pass

# main
def main():
    rospy.init_node('mockMissionPlanner', anonymous=True)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['missionFailed', 'missionSuceeded'])

    # Open the container
    with sm:
        # Add states to the container
        # This is where the  transitions are defined!
        '''
        smach.StateMachine.add('GateState', GateState(), 
                               transitions={'gatePassed':'LaneDetector',
                                            'gateMissed':'missionFailed'})

        smach.StateMachine.add('LaneDetector', LaneDetector(), 
                               transitions={'pointingToNextTask':'SwimStraight',
                                            'notSeeingLane':'missionFailed'})

        smach.StateMachine.add('SwimStraight', SwimStraight(), 
                               transitions={'atNextTask':'',
                                            'notAtNextTask':'missionFailed'})
        '''

        # Gate task. Transitions Directly to pinger task...for now.
        smach.StateMachine.add('GateState', GateState(), 
                               transitions={'gatePassed':'NavitageToSurfacingTask',
                                            'gateMissed':'missionFailed'})
        
        # The surface task. Finishes up the mission.
        smach.StateMachine.add('NavitageToSurfacingTask', NavitageToSurfacingTask(), 
                        transitions={'missionSuceeded':'missionSuceeded'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()