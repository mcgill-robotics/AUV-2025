import rospy
import smach
import actionlib # No ActionServer implemented yet, we might want TODO this eventually

from std_msgs.msg import Bool, Float64

class NavigateToSurfacing(smach.State):

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