#!/usr/bin/env python3

import rospy
import numpy as np

from hydrophones_util import estimate_bearing_planar
from signal import Signal

TARGET_FREQ = 20000 # listen to pings of 20kHz

# TODO - put methods of Signal inside of Hydrophone class
class Hydrophone:
    def __init__(self, x, y, z=0):
        self.pos = np.array([x, y, z])
        self.signal # most recent buffered signal/time history
        self.signal_age = None # make sure we are comparing the same signal 
        self.th = None # discrete time history (pressure vs. time)


'''
Generalized functions for preforming cross correlation and time delay
calculations from hydrophones

'''

'''
returns list - cross-correlations [c_11, c_12, c_13] relative to <ref>
<ref> is taken as the first hydrophone in the list

hydrophones - <list/tuple>: [h1, h2, h3]
'''
def xcorr(hydrophones):
    correlations = []
    ref = hydrophones[0]
    for h in hydrophones:
        correlations.append(np.correlate(h.th, ref.th, 'full'))
    return correlations
    
'''
returns list - signal delays (seconds) relative to <ref> hydrophone
<ref> is taken as the first hydrophone in the list

correlations - <list/tuple>: [c_11, c_12, c_13]
'''
def get_delays_from_xcorr(corr):
    global dt
    delays = []
    i_ref = np.argmax(corr[0])
    for c in corr:
        delays.append((np.argmax(c)-i_ref)*dt)
    return delays
    
    
'''
returns list - signal delays (seconds) relative to <ref> hydrophone
<ref> is taken as the first hydrophone in the list (delays relative 
to this hydrophone's readings)

hydrophones - <list/tuple>: [h1, h2, h3]
'''
def get_delays(hydrophones):
    corr = xcorr(hydrophones)
    return get_delays_from_xcorr(corr)


'''
returns the unit bearing vector,
- assuming the wave is planar

#TODO - assumes configuration of 3 hydrophones at right angles
delays - <list/tuple>:      [dt_11, dt_12, dt_13]
hydrophones - <list/tuple>: [h1, h2, h3]
'''
def estimate_bearing_planar(delays, hydrophones):
    global c

    dt_11, dt_12, dt_13 = delays
    h1, h2, h3 = hydrophones

    # TODO - in general find pair of hydrophones with
    # largest dx, dy
    dx = np.linalg.norm(h2.pos_est[0] - h1.pos_est[0])
    dy = np.linalg.norm(h3.pos_est[1] - h1.pos_est[1])

    # velocity of planar wave: u = u_x*i + u_y*j + u_z*k
    u_x = dt_12/dx#dx/dt_12
    u_y = dt_13/dy#dy/dt_13

    # negative if planar assumption leads to large errors
    u_z_sqr = c**2 - u_x**2 - u_y**2
    u_z = 0#np.sqrt(u_z_sqr) if u_z_sqr > 0 else 0
    u = np.array([u_x, u_y, u_z])

    # bearing to pinger is in opposite direction of wave propagation
    return -u/np.linalg.norm(u)


'''
Estimate bearing using trilateration
- if r1 is assumed to be known the pinger can be triangulated
- r2 = r1 + dt_12*c, r3 = r1 + dt_13*c
- finding intersection of 3 spheres radius r1, r2, r3
- scan over possible r1 to find bearing
- *note: [r1, r2, r3] is ordered same as [dt_11, dt_12, dt_13]
ie. it is not possible to have r1 < r2 if dt_11 > dt_12
since dt_11 > dt_12 => (0 > positive)
'''

'''
this function was taken from
https://stackoverflow.com/questions/1406375/finding-intersection-points-between-3-spheres
'''
# Find the intersection of three spheres
# P1,P2,P3 are the centers, r1,r2,r3 are the radii
# Implementaton based on Wikipedia Trilateration article.
def trilaterate(P1,P2,P3,r1,r2,r3):
    temp1 = P2-P1
    e_x = temp1/np.linalg.norm(temp1)
    temp2 = P3-P1
    i = np.dot(e_x,temp2)
    temp3 = temp2 - i*e_x
    e_y = temp3/np.linalg.norm(temp3)
    e_z = np.cross(e_x,e_y)
    d = np.linalg.norm(P2-P1)
    j = np.dot(e_y,temp2)
    x = (r1*r1 - r2*r2 + d*d) / (2*d)
    y = (r1*r1 - r3*r3 -2*i*x + i*i + j*j) / (2*j)
    temp4 = r1*r1 - x*x - y*y
    if temp4<0:
        raise Exception("The three spheres do not intersect!");
    z = np.sqrt(temp4)
    p_12_a = P1 + x*e_x + y*e_y + z*e_z
    p_12_b = P1 + x*e_x + y*e_y - z*e_z
    return p_12_a,p_12_b


'''
Assumes 3 hydrophones, returns pinger location(s)
'''
def estimate_pinger_trilateration(delays, hydrophones):
    h1, h2, h3 = hydrophones
    dt_11, dt_12, dt_13 = delays

    dt_min = np.min(delays)
    r1_lb = c*dt_min if dt_min < 0 else 0
    r1_ub = None
    tol = 0.1
    window = 0.6 # dummy value that is greater than tol

    r1 = 10 # random initial guess
    p1 = p2 = None
    while window > tol and r1 < 1000:
        #print("r1 guess:", r1)
        #print("window:", window)
        r2 = r1 + dt_12*c
        r3 = r1 + dt_13*c

        try:
            p1, p2 = trilaterate(
            h1.pos_est, h2.pos_est, h3.pos_est,
            r1, r2, r3)
            #print("p1", p1, "p2", p2)

            # r1 is too big
            r1_ub = r1
            r1 = r1_lb + (r1_ub-r1_lb)/2
        except:
            # r1 is too small
            r1_lb = r1
            if r1_ub is None:
                r1*=2
            else:
                r1 = r1_lb + (r1_ub-r1_lb)/2

        window = 0.6 if r1_ub is None else r1_ub-r1_lb

    if p1 is None:
        raise Exception("does not converge")
    # pinger is chosen based on the assumption that it is at the
    # bottom of the pool, say here the more negative z coordinate
    return p1 if p2 is None or p1[2] < p2[2] else p2


def buffer_signals(hydrophones):
    for h in hydrophones:
        h.signal.buffer_signals()




# every time the buffer is updated, a new estimate of the 
# bearing is acquired and published
# this may be updated by the signal?
def update_bearing_estimate():
    bearing = estimate_bearing_planar(signal.delays, hydrophones)
    pub.publish(bearing)


if __name__ == '__main__':
    rospy.init_node('hydrophones')
    # TODO - change to actual positions
    h1 = Hydrophone(0, 0)
    h2 = Hydrophone(1, 0)
    h3 = Hydrophone(0, 1)
    hydrophones = [h1, h2, h3]
    
    # bearing estimate in robot's reference frame
    pub = rospy.Publisher('/hydrophones/bearing_est', Vector3)
    signal = Signal(TARGET_FREQ)

    # TODO: the signal should be buffered at the ping and an interrupt triggered
    # timer = rospy.Timer(rospy.Duration(0.5), signal.new_signal)
    
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
