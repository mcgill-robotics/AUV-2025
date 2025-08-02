#!/usr/bin/env python3
import rospy
from auv_msgs.msg import ThrusterMicroseconds

# Define PWM values for different movements
#I will use proper naming conventions and coordinate frames in the future, pls dont shoot me im really tired rn  
FORWARD_PWMS = [1700, 1500, 1500, 1230, 1250, 1500, 1500, 1700] #from last pool test, these make robot very straight
U_FORWARD_PWMS= [1700, 1685, 1595, 1283, 1300, 1590, 1675, 1700]
STOP_PWMS = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500] 
TURN_LEFT_PWMS = [1500, 1500, 1500, 1230, 1500, 1500, 1500, 1500] # YAW TODO: adjust this pls
DOWN_PWMS = [1500, 1637, 1637, 1500, 1500, 1600, 1600, 1500] 
PITCHUP_PWMS = [1500, 1650, 1500, 1500, 1500, 1500, 1660, 1500] #change this

def publish_pwm_for_duration(publisher, msg, duration, rate):
    """
    Helper function to publish specific PWM values for a given duration.
    """
    rospy.loginfo(f"{msg.microseconds} for {duration} seconds")
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
        publisher.publish(msg)
        rate.sleep()
  
def prequal():
    rospy.init_node("prequal")

    pub = rospy.Publisher('/propulsion/microseconds', ThrusterMicroseconds, queue_size=10)

    rate = rospy.Rate(10) # Publishing rate: 10 Hz, increase for smoother control

    rospy.loginfo("Starting prequal publisher...")

    # Create a ThrusterMicroseconds message for various movements
    msg_forward = ThrusterMicroseconds(microseconds=U_FORWARD_PWMS)
    msg_turn_left = ThrusterMicroseconds(microseconds=TURN_LEFT_PWMS)
    msg_stop=ThrusterMicroseconds(microseconds=STOP_PWMS)
    msg_down=ThrusterMicroseconds(microseconds=DOWN_PWMS)
    msg_pitchup=ThrusterMicroseconds(microseconds=PITCHUP_PWMS)

    ###ACTUAL PREQUAL RUN ###
    #TODO: adjust the actual duration
    #the stopping PWMs may be completely unnecessary, remove if not needed
    #Also that we cannot submerge. If we need to stay underwater, 
    # replace STOP by [1500, x, x, 1500, 1500, x, x, 1500] where x is whatever value
    # and add that same x for the forward and left PWMs
    publish_pwm_for_duration(pub, msg_down, 3.0, rate)
    publish_pwm_for_duration(pub, msg_forward, 15.0, rate)
    publish_pwm_for_duration(pub, msg_stop, 2.0, rate)


if __name__ == '__main__':
    try:
        prequal()
    except rospy.ROSInterruptException:
        rospy.loginfo("epic failure")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")


#HOW TO RUN THIS FILE (if you dont know):
# roslaunch propulsion propulsion.launch
# rosrun planner raw_prequal.py