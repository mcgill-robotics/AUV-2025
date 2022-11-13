#!/usr/bin/env python3

import rospy
import math
from auv_msgs.msg import ThrusterForces, ThrusterIntensities

MAX_FORCE_FWD = 5.25*9.81 #max theoretical force to expect (in newtons) for going forward
MAX_FORCE_BKWD = -4.1*9.81 #max theoretical force to expect (in newtons) for spinning backward

if __name__ == '__main__':
    rospy.init_node('force_to_intensity')
    publisher = rospy.Publisher('/propulsion/thruster_intensities', ThrusterIntensities, queue_size=5)

    def force_to_intensity(force):
        motor_thrust = 0
        # cap our input force at maximum fwd/bkwd speeds
        force = min(max(force, MAX_FORCE_BKWD), MAX_FORCE_FWD)
        #get correct max force
        if force >= 0:
            MAX_FORCE = MAX_FORCE_FWD
        else:
            MAX_FORCE = MAX_FORCE_BKWD
        return math.log2((force/MAX_FORCE)+1)

    def forces_to_intensities(f):
        # After a quick google search i found that approximately thrust = rpm^2, so i wanted to test scaling
        # the intensities with log2 rathern than keeping them spread linearly from 0 to 1. I also thought that it could help
        # remove noise/small unnecessary movements of motors and make movements faster for medium-low strength forces
        out = [0]*8
        out[ThrusterIntensities.SURGE_PORT] = force_to_intensity(f.SURGE_PORT)
        out[ThrusterIntensities.SURGE_STAR] = force_to_intensity(f.SURGE_STAR)
        out[ThrusterIntensities.SWAY_BOW] = force_to_intensity(f.SWAY_BOW)
        out[ThrusterIntensities.SWAY_STERN] = force_to_intensity(f.SWAY_STERN)
        out[ThrusterIntensities.HEAVY_BOW_PORT] = force_to_intensity(f.HEAVY_BOW_PORT)
        out[ThrusterIntensities.HEAVY_BOW_STAR] = force_to_intensity(f.HEAVY_BOW_STAR)
        out[ThrusterIntensities.HEAVY_STERN_STAR] = force_to_intensity(f.HEAVY_STERN_STAR)
        out[ThrusterIntensities.HEAVY_STERN_PORT] = force_to_intensity(f.HEAVY_STERN_PORT)
            
        out = ThrusterIntensities(out)
        publisher.publish(out)

    subscriber = rospy.Subscriber('/propulsion/thruster_forces', ThrusterForces, forces_to_intensities)
    rospy.spin()
