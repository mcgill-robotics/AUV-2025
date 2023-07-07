#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import Vector3

TARGET_FREQ = 20000 # listen to pings of 20kHzi #TODO - variable

# natural constants
c = 1490 # m/s speed of sound in water - TODO - more precise?



class Hydrophone:
    def __init__(self, x, y, z=0):
        self.pos = np.array([x, y, z])
        self.delay = 0.0 # signal time delay (t_h1 - t_hx) 


class PingerFinder:
    def __init__(self, target_freq):
        # TODO - change to actual positions
        self.h1 = Hydrophone(0, 0)
        self.h2 = Hydrophone(1, 0)
        self.h3 = Hydrophone(0, 1)
        self.hydrophones = [h1, h2, h3]


        # keep track of history of (global) positions and 
        # bearing estimates to estimate pinger location
        self.auv_positions = []
        self.bearings = []

        # TODO - instead of keeping track of entire history, 
        # have rolling best estimate and weight for next available
        # measurement (num measurements)
        self.pinger_location = np.array([0, 0, 0])

        self.pub_bearing = rospy.Publisher('/hydrophones/bearing_est', Vector3)
        rospy.Subscriber('/hydrophones/delays', HydrophoneDelays, self.delays_cb) # TODO - hydrophone message


    def delays_cb(self, msg):
        # TODO - example message:
        # uint freq
        # float64 dt_12 # t_h1 - t_h2
        # float dt_13 # t_h1 - t_h3
        if msg.freq == TARGET_FREQ:
            self.h2.delay = msg.dt_12
            self.h3.delay = msg.dt_13


    '''
    returns the unit bearing vector,
    - assuming the wave is planar

    #TODO - assumes configuration of 3 hydrophones at right angles
    delays - <list/tuple>:      [dt_11, dt_12, dt_13]
    hydrophones - <list/tuple>: [h1, h2, h3]
    '''
    def estimate_bearing_planar(self):
        # TODO - in general find pair of hydrophones with
        # largest dx, dy
        dx = np.linalg.norm(self.h2.pos[0] - self.h1.pos[0])
        dy = np.linalg.norm(self.h3.pos[1] - self.h1.pos[1])

        # velocity of planar wave: u = u_x*i + u_y*j + u_z*k
        u_x = self.h2.delay/dx #dx/dt_12
        u_y = self.h3.delay/dy #dy/dt_13

        # negative if planar assumption leads to large errors
        # TODO - check how to find z direction
        u_z_sqr = c**2 - u_x**2 - u_y**2
        u_z = 0#np.sqrt(u_z_sqr) if u_z_sqr > 0 else 0
        u = np.array([u_x, u_y, u_z])

        # bearing to pinger is in opposite direction of wave propagation
        return -u/np.linalg.norm(u)


    def update_bearing(self):
        # TODO - consolidate, might fuse multiple methods?
        bearing = self.estimate_bearing_planar()

        # TODO - account for mounting of array
        self.pub_bearing.publish(bearing) #TODO - check auto type conversion


    def estimate_location(self):
        # TODO - estimate pinger location beased
        # on the position history of bearing estimates



if __name__ == '__main__':
    pf = PingerFinder(TARGET_FREQ) # TODO - various possible target signal frequencies
    timer = rospy.Timer(rospy.Duration(0.1), pf.update_bearing)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
