#!/usr/bin/env python3

import rospy
from auv_msgs.msg import ThrusterForces, ThrusterIntensities


# forces produced by T200 thruster at 16V (N)
MAX_FWD_FORCE = 5.25*9.81 
MAX_BKWD_FORCE = 4.1*9.81

def force_to_intensity(force):
        motor_thrust = 0
        # cap our input force at maximum fwd/bkwd speeds
        force = min(max(force, -MAX_FORCE_BKWD), MAX_FORCE_FWD)
        if force > 0:
            max_log = math.log2(MAX_FORCE_FWD) #get log value of maximum fwd force
            return (math.log2(force)+2+stiffness)/(max_log+2+stiffness) #+2 to make the logs start at 0
        elif force < 0:
            max_log = math.log2(MAX_FORCE_BKWD) #get log value of maximum bkwd force
            return -(math.log2(-force)+2+stiffness)/(max_log+2+stiffness) #we negate the final return value since we want a negative intensity backwards
        else: return 0


pub = rospy.Publisher('/propulsion/thruster_intensities', ThrusterIntensities, queue_size=5)


# functions or relevant code
def forces_to_intensities_cb(forces_msg):

    # array message to publish
    intens_arr = [None]*8
    intens_arr[ThrusterIntensities.SURGE_PORT] = force_to_intensity(forces_msg.SURGE_PORT)
    intens_arr[ThrusterIntensities.SURGE_STAR] = force_to_intensity(forces_msg.SURGE_STAR) 
    intens_arr[ThrusterIntensities.SWAY_BOW] = force_to_intensity(forces_msg.SWAY_BOW)
    intens_arr[ThrusterIntensities.SWAY_STERN] = force_to_intensity(forces_msg.SWAY_STERN) 
    intens_arr[ThrusterIntensities.HEAVE_BOW_PORT] = force_to_intensity(forces_msg.HEAVE_BOW_PORT) 
    intens_arr[ThrusterIntensities.HEAVE_BOW_STAR] = force_to_intensity(forces_msg.HEAVE_BOW_STAR) 
    intens_arr[ThrusterIntensities.HEAVE_STERN_STAR] = force_to_intensity(forces_msg.HEAVE_STERN_STAR) 
    intens_arr[ThrusterIntensities.HEAVE_STERN_PORT] = force_to_intensity(forces_msg.HEAVE_STERN_PORT)

    intens_msg = ThrusterIntensities(intens_arr)
    pub.publish(intens_msg)
    


if __name__ == '__main__':
    rospy.init_node('force_to_intensity')
    rospy.Subscriber('/propulsion/thruster_forces', ThrusterForces, forces_to_intensities_cb)
    rospy.spin()
