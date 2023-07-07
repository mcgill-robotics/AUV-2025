#!/usr/bin/env python3

'''
This file is for dry-testing the hydrophones by simulating a signal
'''

import numpy as np
import hydrophones.py

c = 3 # m/s speed of sound in water
t_final = 5
dt = 0.005
n = int(t_final/dt)
t_arr = np.linspace(0, t_final, n+1)

A = 10
phi = 0                 # rad
f = 0.5                 # hz
w = 2*np.pi*f
t2 = 2                  #s


def signal(t):
    if t < 0 or t > t2:
        return 0
    return A*np.sin(w*t - phi)


class Pinger:
    def __init__(self, x, y, z=0):
        self.pos = np.array([x, y, z])
        self.signal = np.frompyfunc(signal, 1, 1)   


'''
Time histories of signal as experienced at each of the hydrophones
'''
# TODO


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

