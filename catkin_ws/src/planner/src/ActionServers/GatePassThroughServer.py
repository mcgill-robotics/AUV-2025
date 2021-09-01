import rospy
import actionlib
import time

from std_msgs.msg import Float64, String
from planner.msg import GatePassThroughAction, GatePassThroughFeedback, GatePassThroughResult

class GatePassThroughServerClass():

    def __init__(self):

        # Constants
        self.COUNTS_FOR_STABILITY = 2 # If looking for gate

        # Constants from goal
        self.SURGE_TIME = 0 # seconds
        self.SURGE_MAGNITUDE = 0 # ?

        # Data variables
        self.stable_counts = 0 # If looking for gate

        # Surge publisher
        self.surge_magnitude_pub  = rospy.Publisher('/controls/superimposer/surge'   , Float64 , queue_size=1)

        # Define the action server, and start it
        self._action_name = 'GatePassThrough_Action'
        self._as = actionlib.SimpleActionServer(self._action_name, GatePassThroughAction,
                                                     execute_cb=self.execute_cb, auto_start = False)
        self._as.start()


    def execute_cb(self, goal):
        rospy.loginfo('Passing through gate')

        # Set a variable that will remain true unless the server is pre-empted
        success = True
        
        print("Goal:")
        print(goal)

        # Unpack the goal message into its components
        self.SEARCH_FOR_GATE = goal.search_for_gate.data
        self.SURGE_MAGNITUDE = goal.surge_magnitude.data
        self.SURGE_TIME = goal.surge_time.data

        if goal.search_for_gate.data:
            print("Looking for gate")
            # Implement logic for looking for gate
            # Would require:
            # - Turning and possibly moving to be within gate (need PIDs - publishers)
            # - CV stuff (need subscribers)
            pass

        print("Surging to gate")

        surge_start_time = rospy.get_time()
        self.surge_magnitude_pub.publish(self.SURGE_MAGNITUDE) # Start surging
        for x in range(self.SURGE_TIME):
            time_string = "Surging for {} more second(s)".format(self.SURGE_TIME - x)
            print(time_string)
            self._as.publish_feedback(GatePassThroughFeedback(current_action = String(time_string)))
            time.sleep(1)
        
        self.surge_magnitude_pub.publish(0) # Stop surging

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(result = GatePassThroughResult(distance_traveled = Float64(0))) # Hardcoded for now. We may wish to implement if we get the DVL to work
        
        return


if __name__ == '__main__':
    rospy.init_node('GatePassThroughServer_Node')
    server = GatePassThroughServerClass()
    rospy.spin()   