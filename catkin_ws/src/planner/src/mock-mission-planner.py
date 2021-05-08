import rospy
import smach
from std_msgs.msg import Bool, Float64

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
    The gate state
    '''
    def __init__(self):
        smach.State.__init__(self, outcomes=['gatePassed', 'gateMissed'])
        self.counter = 0
    def execute(self, userdata):
        rospy.loginfo('Trying to pass through the gate')
        # publish PID setpoint?
        # Check if we are stable at the setpoint? 
        # 
        if self.counter < 3:
            return 'gatePassed'
        
        else:
            rospy.loginfo("We missed the gate")
            return 'gateMissed'




# define state LaneDetector
class LaneDetector(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pointingToNextTask', 'notSeeingLane'])

    def execute(self, userdata):
        rospy.loginfo('Executing state LaneDetector')

        if seeingLane: # !!! this condition is not defined
            return 'pointingToNextTask'
        
        else:
            return 'notSeeingLane'

class SwimStraight(smach.State):
    def __init__(self):
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
        self.YAW_THRESHOLD      = 20 * 3.14/180 # 20 degrees, but converted to Radians, threshold for being "aligned" to the pinger
        self.ARRIVAL_THRESHOLD  = 50 * 3.14/180 #  Threshold for checking if the pinger is "behind" us and we have arrived 
        self.STABLE_COUNT       = 5             # This defines the number of measurements we need to take within the yaw threshold to be stable
        self.alignment_count    = 0
        self.arrival_count      = 0
        self.ever_aligned       = False         # A check if we have ever been aligned to the pinger.
        self.YAW_TARGET         = 0             # This will depend on how the hydrophones are mounted, I am guessing 0 for now. 
        self.SURGE_MAGNITUDE    = 1             # I have no idea what the units are, be careful.
        self.successful_surface = False

        #Set up all of the publishers I will use later
        self.yaw_enable_pub      = rospy.Publisher('/controls/yaw_pid/pid_enable' , Bool    , queue_size=1)
        self.yaw_setpoint_pub    = rospy.Publisher('/controls/yaw_pid/setpoint'   , Float64 , queue_size=1)
        
        self.surge_magnitude_pub = rospy.Publisher('/controls/superimposer/surge' , Float64 , queue_size=1)

        self.depth_enable_pub    = rospy.Publisher('/controls/depth_pid/pid_enable' , Float64 , queue_size=1)
        
        #Enable the yaw PID, try to align to the pinger.
        yaw_setpoint_pub(YAW_TARGET)  
        yaw_enable_pub.publish(1)

    ''' Considering doing this in an encapsulated manner to improve readability. I think it might get....less readable this way.

    def is_aligned(self):
        return self.heading < self.YAW_THRESHOLD

    def is_stable(self):
        return self.current_count > self.alignment_counts

    def is_arrived(self):
         return (self.ever_aligned and self.heading > ARRIVAL_THRESHOLD)

    def is_stable_at_arrival(self):
        return self.arrival_count > self.STABLE_COUNT
                
    def hydrophones_cb(self, msg):
        self.heading = msg # takes the heading float 64 from the subscriber and sets it to a variable

        if self.is_aligned():
            if self.is_stable():
                surge_magnitude_pub.publish(SURGE_MAGNITUDE)
                self.ever_aligned = True # We've been aligned once. Now if we get unaligned it may be due to arrival
            else:
                #aligned but not yet stable
                self.alignment_count += 1
        else:
            #not aligned
            self.alignment_count = 0
            surge_magnitude_pub.publish(0) # We are very unaligned. Stop and re-align.

            # if we're not aligned, we might have arrived! Check for this
            if self.is_arrived():
                if self.is_stable_at_arrival():
                    self.depth_enable_pub(0) 
                else:
                    self.arrival_count +=1 
    '''

    def hydrophones_cb(self, msg):
        self.heading = msg # takes the heading float 64 from the subscriber and sets it to a variable
        # Check if we are stable! If we are outside the threshold, restart the counter.

        #------------------------------------- Checking for alignment to the pinger -------------------------------------
        if self.heading < self.YAW_THRESHOLD :
            self.alignment_count += 1
        else:
            self.alignment_count  = 0

        if self.alignment_count > self.STABLE_COUNT : # We are aligned, go forward
            surge_magnitude_pub.publish(SURGE_MAGNITUDE) 
        else :
            surge_magnitude_pub.publish(0) # We are very unaligned. Stop and re-align.

        #------------------------------------- Checking for arrival at the pinger -------------------------------------
        # If ever aligned and now outside the arrival threshold?
        if (self.ever_aligned & self.heading > ARRIVAL_THRESHOLD) :
            # Check if this is stable
            surge_magnitude_pub.publish(0) #First, turn off thrusters
            self.arrival_count +=1
            if arrival_count > STABLE_COUNT :
                #If it is stable, surface!
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

        rospy.loginfo('Executing state Navigateto surfacing task')
        if successful_surface: # !!! this condition is not defined
            return 'missionSuceeded'
        else:
            pass

# main
def main():
    rospy.init_node('mock-mission-planner')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['missionFailed', 'missionSuceeded'])

    # Open the container
    with sm:
        # Add states to the container
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

        smach.StateMachine.add('GateState', GateState(), 
                               transitions={'gatePassed':'NavitageToSurfacingTask',
                                            'gateMissed':'missionFailed'})

        smach.StateMachine.add('NavitageToSurfacingTask', NavitageToSurfacingTask(), 
                        transitions={'surfaced':'missionSuceeded'})



    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()