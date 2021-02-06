import rospy
import smach

# define state GateState
class GateState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['gatePassed', 'gateMissed'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Trying to pass through the gate')
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

# main
def main():
    rospy.init_node('mock-mission-planner')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['missionFailed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GateState', GateState(), 
                               transitions={'gatePassed':'LaneDetector',
                                            'gateMissed':'missionFailed'})
        smach.StateMachine.add('LaneDetector', LaneDetector(), 
                               transitions={'pointingToNextTask':'SwimStraight',
                                            'notSeeingLane':'missionFailed'})
        smach.StateMachine.add('SwimStraight', SwimStraight(), 
                               transitions={'atNextTask':'',
                                            'notAtNextTask':'missionFailed'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()