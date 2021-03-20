#!/usr/bin/env python

import roslib; 
import rospy
import smach
import smach_ros

from planner.msg import NavigateAction, NavigateGoal

from actionlib import *
from actionlib_msgs.msg import *


# Create a Navigation action server
class NavigateServer:
    '''
    This server is instantiated whenever we want to navigate somewhere.
    The
    '''
    def __init__(self,name):
        '''
        Initializes a simple action server. 
        The syntax here is "server name" , "action name" , "callback"
        '''
        self._sas = SimpleActionServer(name,
                NavigateAction,
                execute_cb=self.execute_cb)

    def execute_cb(self, msg, userdata):
        '''
        Publishes a setpoint messages to the controls system
        Waits for the robot to be at the setpoint for a pre-determined number of "counts"

        '''
        rospy.loginfo('Executing Navigate state')
        rospy.loginfo('Current z position = %f'%userdata.currentz)

        if (msg.ztarget - userdata.currentz) == 0:
            rospy.loginfo('At the target')
            self._sas.set_succeeded()
        else:
            rospy.loginfo('NOT at target, waiting')
            #presend to wait for the control system
            #For now, just set the current z to the  target, as if the control system is working.
            userdata.currentz = msg.ztarget

            #To implement: if pre-empted, hold current position. 

#Create a lane detector action server


# define state LaneDetector
class InitializeState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['suceeded', 'failed'])

    def execute(self, userdata):
        #Dummy task. Suceeds and exits the state machine
        rospy.loginfo('Executing initialize State')
        return 'suceeded'

class GateState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['suceeded', 'failed'])

    def execute(self, userdata):
        #Dummy task. Suceeds and exits the state machine
        rospy.loginfo('Executing initialize State')
        return 'suceeded'

def main():
    rospy.init_node('smach_example_actionlib')

    # Start an action server
    server = NavigateServer('navigate_action')

    # Create a SMACH state machine
    sm0 = smach.StateMachine(outcomes=['Mission_succeeded','Mission_aborted','Mission_preempted'])
    sm0.userdata.currentx = 5
    sm0.userdata.currenty = 5
    sm0.userdata.currentz = 5

    # Open the container
    with sm0:
        # Add states to the container

        # Add a simple action state. This will use an empty, default goal
        # As seen in TestServer above, an empty goal will always return with
        # GoalStatus.SUCCEEDED, causing this simple action state to return
        # the outcome 'succeeded'
        smach.StateMachine.add('Gate_State',
                               smach_ros.SimpleActionState( 'navitate_action', 
                                                            TestAction,
                                                            goal = TestGoal(ztarget=1)), #name of the action server, name of the action "message" to be passed around                                                     
                               transitions = {'succeeded':'Lane_State'})

        # Add another simple action state. This will give a goal
        # that should abort the action state when it is received, so we
        # map 'aborted' for this state onto 'succeeded' for the state machine.
        smach.StateMachine.add('Lane_State',
                               transitions = {'succeeded':'Mission_suceeded'},)

        
        '''
        # Add another simple action state. This will give a goal
        # that should abort the action state when it is received, so we
        # map 'aborted' for this state onto 'succeeded' for the state machine.
        def goal_callback(userdata, default_goal):
            goal = TestGoal()
            goal.order = 2
            return goal

        smach.StateMachine.add('Callback_State',
                               smach_ros.SimpleActionState('test_action', TestAction,
                                                       goal_cb = goal_callback),
                               {'aborted':'succeeded'})

        # For more examples on how to set goals and process results, see 
        # executive_smach/smach_ros/tests/smach_actionlib.py
        '''

    # Execute SMACH plan
    outcome = sm0.execute()

    rospy.signal_shutdown('All done.')


if __name__ == '__main__':
    main()