# -*- coding: utf-8 -*-

"""Pinger model."""

import rospy
import numpy as np
from scipy import signal as sp

__author__ = "Anass Al-Wohoush"


def generate_signal(buffersize, target_freq, fs, time_offset=0.00,
                    period=2.0, pulse_length=4e-3, snr=20, sweep=200):
    """Yields time shifted signal.

    Args:
        buffersize: Buffersize.
        target_freq: Target frequency in Hz.
        fs: Sampling frequency in Hz.
        time_offset: Time offset in seconds (default: 0 s).
        period: Time between pings in seconds (default: 2 s).
        pulse_length: Pulse length in seconds (default: 4 ms).
        snr: Signal-to-noise ratio in dB (default: 20 dB).
        sweep: Frequency sweep in Hz (default: 200 Hz).

    Yields:
        Signal of buffersize.
    """
    # Convert from seconds to units of sampling period.
    next_ping = int(np.ceil(time_offset * fs)) + buffersize / 2
    ping_length = int(round(pulse_length * fs))
    ping_period = int(np.ceil(period * fs))

    # Generate perfect ping with linear chirp.
    time = np.arange(ping_length) / float(fs)
    ping = 10 ** (snr / 20) * sp.chirp(
        t=time, t1=time[ping_length / 4],
        f0=max(0, target_freq - 200), f1=target_freq,
        phi=90
    )

    # Keep track of how far in a ping we are.
    current_ping_index = 0
    pinging = False

    while not rospy.is_shutdown():
        # Setup random signal of the correct buffersize.
        signal = np.random.normal(0, 1, buffersize)

        for i, s in enumerate(signal):
            # Decrement time until next ping.
            next_ping -= 1

            # Set that the next ping is about to start.
            if next_ping < 1:
                current_ping_index -= next_ping
                next_ping += ping_period
                pinging = True

            # Verify if ping is complete.
            if current_ping_index >= ping_length:
                current_ping_index = 0
                pinging = False

            # Add ping to signal.
            if pinging:
                signal[i] = s + ping[current_ping_index]
                current_ping_index += 1

        # Yield current signal.
        yield signal.tolist()
