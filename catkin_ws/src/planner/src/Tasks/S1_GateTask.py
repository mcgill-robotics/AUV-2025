import rospy
import smach
import actionlib # No ActionServer implemented yet, we might want TODO this eventually
import time

from blinky.msg import TaskStatus # Blinky commented out for now TODO
from std_msgs.msg import Bool, Float64
from planner.msg import DepthAction, DepthGoal, DepthResult, GatePassThroughAction, GatePassThroughGoal, GatePassThroughResult

class GateTask(smach.State):
    # The gate state : ded reckoning
    # 0) Read the depth 
    # 1) Submerge to a known depth
    # 2) Check for stability at this depth
    # 3) surge forward for an empircally determined amount of time. (yikes.)
    # 4) Transitions to the next state

    def __init__(self):

        # Initialize class as state and define transitions
        smach.State.__init__(self, outcomes=['gatePassed', 'gateMissed', 'gatePreempted'])

        # Abstract constants
        self.COUNTS_FOR_STABILITY  = 2

        # Depth constants
        self.DEPTH_SETPOINT        = 2.5 # meters
        self.DEPTH_THRESHOLD       = 0.2 # meters

        # Surge constants
        self.SURGE_MAGNITUDE       = 1   # one day I'll know what these units are. Start small and increment in pool testing.
        self.SURGE_DURATION        = 10  # seconds

        # Define the publisher for blinky, publish a task...just a random one for now
        #self.blinky_pub            = rospy.Publisher('/task'   , TaskStatus , queue_size=1)
        #self.taskmsg        = TaskStatus()
        #self.taskmsg.action = 1
        #self.taskmsg.task   = 1
        #self.blinky_pub.publish (taskmsg)


    def execute(self, userdata):

        rospy.loginfo('Inside Gate State')

        # Check if we are stable at the setpoint? 
        while (not self.done_surging): # Loop inside here untill we are ready to move to the next state

            if self.preempt_requested():
                return 'gatePreempted'

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
'''
        # Depth Action
        print("Starting depth client")
        print("Creating client...")
        client = actionlib.SimpleActionClient('Depth', DepthAction)
        print("Client created, waiting for server...")
        client.wait_for_server()
        print("Found server, creating goal...")
        goal = DepthGoal(depth_target = Float64(self.DEPTH_SETPOINT))
        print("Goal created, sending goal...")
        print(goal)
        client.send_goal(goal)
        print("Goal sent, waiting for result...")
        client.wait_for_result()
        print("Result received")
        print(client.get_result())
        print("Stop centering ActionServer")

        print("Successfully reached depth")

        # GatePassThrough Action
        print("Starting GatePassThrough client")
        print("Creating client...")
        client = actionlib.SimpleActionClient('GatePassThrough_Action', GatePassThroughAction)
        print("Client created, waiting for server...")
        client.wait_for_server()
        print("Found server, creating goal...")
        goal = GatePassThroughGoal(search_for_gate = Bool(False), surge_magnitude = Float64(self.SURGE_MAGNITUDE), surge_time = Float64(self.SURGE_DURATION))
        print("Goal created, sending goal...")
        print(goal)
        client.send_goal(goal)
        print("Goal sent, waiting for result...")
        client.wait_for_result()
        print("Result received")
        print(client.get_result())
        print("Stop centering ActionServer")

        return 'gatePassed'
'''