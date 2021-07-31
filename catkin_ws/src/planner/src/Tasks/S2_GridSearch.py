import rospy
import smach
import actionlib # No ActionServer implemented yet, we might want TODO this eventually

from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Vector3Stamped

'''
!!! This task needs work !!!
'''

class GridSearch(smach.State):
    # This Executes a raster scan to search for something, probably a lane
    # 1) Turn 90 degrees using the IMU 
    # 2) Surge for half the predetermined time (long)
    # 3) Turn 180 degrees using the IMU
    # 4) Surge for a predetermined time (long)

    # loop  following steps until we find lane or reach max_value
    # 5) Turn 90 degrees using the IMU
    # 6) Surge for a predetermined time (short)
    # 7) Turn 90 degrees using the IMU
    # 8) Surge for a predetermined time (long), call lane detector after set time
    # 9) Change sign of 90 degrees turn
        
    def __init__(self):

        # Initialize class as state and define transitions
        smach.State.__init__(self, outcomes=['missionSucceeded'])

        # Abstract constants
        self.COUNTS_FOR_STABILITY      = 1

        # Search constants
        self.SHORT_TIME                = 1 # seconds
        self.NUMBER_OF_SHORT_PER_LONG  = 5 
        self.MAX_ITERATION             = 5
        self.YAW_THRESHOLD             = 10 * 3.14 / 180 # radians
        self.SURGE_MAGNITUDE           = 1
        self.TURN_ANGLE                = 90 # degrees

        # Variables
        self.current_yaw               = None 
        self.current_yaw               = 0 # We have the same variable twice?
        self.stable_at_yaw             = False   
        self.done_surging              = False   
        self.current_count             = 0
        self.angle_achieved_time       = None 

        # IMU subscriber
        self.current_pose_sub     = rospy.Subscriber('/robot_state', Vector3Stamped, self.IMU_cb)

        # Yaw PID publishers
        self.yaw_enable_pub       = rospy.Publisher('/controls/yaw_pid/pid_enable', Bool   , queue_size=1)
        self.yaw_setpoint_pub     = rospy.Publisher('/controls/yaw_pid/setpoint'  , Float64, queue_size=1)
        # Yaw PID gets "data" from IMU, no need to pulish from here

        # Surge publisher - publishes magnitude at which we want to surge
        self.surge_magnitude_pub  = rospy.Publisher('/controls/superimposer/surge', Float64, queue_size=1)


    def IMU_cb(self, msg):
        self.current_yaw = msg.vector.z

    def turn_90_degrees(self, direction):
        print("entered turn 90 deg")
        if(direction == 'left'):
            print("goign left")
            target_yaw = self.current_yaw - 3.14/2.0
        elif(direction=='right'):
            print("going right")
            target_yaw = self.current_yaw + 3.14/2.0
        else:
            print("Invalid direction. Choose 'left' or 'right'.")
            return

        self.yaw_setpoint_pub.publish(target_yaw)
        self.yaw_enable_pub.publish(True)
        self.current_count   = 0
        while not self.stable_at_yaw:
            print("inside 90 deg while loop")
            print("target_yaw {}".format(target_yaw))
            if abs(self.current_yaw-target_yaw) < self.YAW_THRESHOLD:
                print('recieved a stable count')
                self.current_count += 1
            else:
                self.current_count = 0

            if self.current_count > self.COUNTS_FOR_STABILITY : # We are at correct yaw angle! 
                print("jolly good")
                self.stable_at_yaw = True   # I am assuming this never needs to be set false. Danger danger. 

    def move_forward(self):
         while ( not self.done_surging): # Loop inside here untill we are ready to move to the next state
            #print out the number of counts we are waiting for
            self.remaining_counts = self.COUNTS_FOR_STABILITY - self.current_count
            if self.remaining_counts > 0 :
                rospy.loginfo_throttle(1, 'Attaining yaw angle: Need {} more stable readings'.format(self.remaining_counts))      

            if self.stable_at_yaw == True:
                #Get the timestamp of the first moment the robot is at depth
                if self.angle_achieved_time is None: 
                    self.angle_achieved_time = rospy.get_time()
                # Surge for a known duration, then exit the state
                self.time_remaining = self.SHORT_TIME - (rospy.get_time() - self.angle_achieved_time)
                if ( self.time_remaining >0):
                    self.surge_magnitude_pub.publish(self.SURGE_MAGNITUDE)
                    rospy.loginfo_throttle(1, 'Surging: Time left is {} seconds'.format(self.time_remaining))
                else:
                    self.surge_magnitude_pub.publish(0)
                    self.done_surging = True
                    return
        

    def execute(self, userdata):
        yaw_before_turn = self.current_yaw
        self.turn_90_degrees('left')
    
        self.move_forward()
        return 'missionSucceeded'
    

        if seeingLane: # !!! this condition is not defined
            return 'pointingToNextTask'
        
        else:
            return 'notSeeingLane'