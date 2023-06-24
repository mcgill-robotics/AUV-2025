#!/usr/bin/env python3

import numpy as np

c = 3 # m/s speed of sound in water
t_final = 5
dt = 0.005
n = int(t_final/dt)
t_arr = np.linspace(0, t_final, n+1)

A = 10
phi = 0                 # rad
f = 0.5                 # hz
w = 2*np.pi*f
t2 = 2 #s

def signal(t):
    if t < 0 or t > t2:
        return 0
    return A*np.sin(w*t - phi)


class Signal:
    def __init__(self, freq):
        self.freq = freq
        self.time_history = np.array([])
        self.is_ready = False

    '''
    TODO: buffers raw signal data from hydrophones  

    how should this operation should happen? 
    - in a separate thread in the background
    '''
    def buffer_signal():
        # get new data from hydrophones
        # filter the data to include only the target frequency
        # call the update signal callback
        # TODO - this is just test data
        usignal = np.frompyfunc(signal, 1, 1)
        self.time_history = usignal(t_arr-t_delay)
        return


    '''
    at competition, there may be several active pingers, 
    we only care about a pinger of a particular frequency
    - ensure we only include 
    '''
    def filter_freq(f):
        return


