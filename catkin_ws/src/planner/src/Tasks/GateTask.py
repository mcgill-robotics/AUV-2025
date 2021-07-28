import rospy
import smach
import actionlib # No ActionServer implemented yet, we might want TODO this eventually
import time
from blinky.msg import TaskStatus # Blinky commented out for now TODO

class GateTask(smach.State):
    # The gate state : ded reckoning
    # 0) Read the depth 
    # 1) Submerge to a known depth
    # 2) Check for stability at this depth
    # 3) surge forward for an empircally determined amount of time. (yikes.)
    # 4) Transitions to the next state

    def __init__(self):

        # Initialize class as state and define transitions
        smach.State.__init__(self, outcomes=['gatePassed', 'gateMissed'])

        # Abstract constants
        self.COUNTS_FOR_STABILITY  = 1

        # Depth constants
        self.DEPTH_SETPOINT        = 2.5 # meters
        self.DEPTH_THRESHOLD       = 0.2 # meters

        # Surge constants
        self.SURGE_MAGNITUDE       = 1   # one day I'll know what these units are. Start small and increment in pool testing.
        self.SURGE_DURATION        = 10  # seconds

        # Variables
        self.stable_counts         = 0   # counter variable to determine if we are stable at the setpoint
        self.stable_at_depth       = False
        self.depth_achieved_time   = None 
        self.done_surging          = False

        # Depth subscriber
        self.depth_sub            = rospy.Subscriber('/state_estimation/depth', Float64, self.depth_cb)

        # Depth PID publisher
        self.depth_enable_pub     = rospy.Publisher('/controls/depth_pid/pid_enable' , Bool    , queue_size=1)
        self.depth_setpoint_pub   = rospy.Publisher('/controls/depth_pid/setpoint'   , Float64 , queue_size=1)

        # Surge publisher
        self.surge_magnitude_pub  = rospy.Publisher('/controls/superimposer/surge'   , Float64 , queue_size=1)

        # Define the publisher for blinky, publish a task...just a random one for now
        #self.blinky_pub            = rospy.Publisher('/task'   , TaskStatus , queue_size=1)
        #self.taskmsg        = TaskStatus()
        #self.taskmsg.action = 1
        #self.taskmsg.task   = 1
        #self.blinky_pub.publish (taskmsg)
        
        # Turn on depth PID
        self.depth_setpoint_pub.publish(self.DEPTH_SETPOINT)
        self.depth_enable_pub.publish(True) 


    def depth_cb(self, msg):

        self.depth = msg.data # takes the depth float 64 from the subscriber and sets it to a variable
        print('inside callback. Current depth: {} '.format(self.depth))

        # Check for stability at depth
        if abs(self.depth - self.DEPTH_SETPOINT) < self.DEPTH_THRESHOLD :
            #print('recieved a stable count')
            self.stable_counts += 1
        else:
            self.stable_counts   = 0

        if self.stable_counts > self.COUNTS_FOR_STABILITY : # We are at depth! 
            self.stable_at_depth = True   # I am assuming this never needs to be set false. Danger danger. TODO


    def execute(self, userdata):

        rospy.loginfo('Inside Gate State')

        # Check if we are stable at the setpoint? 
        while (not self.done_surging): # Loop inside here untill we are ready to move to the next state
            #print out the number of counts we are waiting for
            self.remaining_counts = self.COUNTS_FOR_STABILITY - self.stable_counts
            if self.remaining_counts > 0 :
                rospy.loginfo_throttle(1, 'Attaining Depth: Need {} more stable readings'.format(self.remaining_counts))      

            if self.stable_at_depth == True:
                #Get the timestamp of the first moment the robot is at depth
                if self.depth_achieved_time is None: 
                    self.depth_achieved_time = rospy.get_time()
                # Surge for a known duration, then exit the state
                self.time_remaining = self.SURGE_DURATION - (rospy.get_time() - self.depth_achieved_time)
                if (self.time_remaining > 0):
                    self.surge_magnitude_pub.publish(self.SURGE_MAGNITUDE)
                    rospy.loginfo_throttle(1, 'Surging: Time left is {} seconds'.format(self.time_remaining))
                else:
                    self.surge_magnitude_pub.publish(0)
                    self.done_surging = True
                    return 'gatePassed'