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
        smach.State.__init__(self, outcomes=['gatePassed', 'gateMissed'])

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