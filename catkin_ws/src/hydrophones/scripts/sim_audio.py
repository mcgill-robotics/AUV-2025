#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Hydrophones signal generator."""

import rospy
from hydrophones import Mic
from datetime import datetime, timedelta
from hydrophones.freq import get_frequency_energy
from hydrophones.msg import Signal, SignalArrayStamped
from hydrophones.sim import Pinger, compute_tdoa, generate_signal

__author__ = "Anass Al-Wohoush"

mics = [
    Mic((0.00, 0.00, 0.00)),
    Mic((0.00, 0.01, 0.00)),
    Mic((0.01, 0.01, 0.00)),
    Mic((0.01, 0.00, 0.00))
]
pinger = Pinger((-10.00, 2.00, 4.00))

fs = 2e5
speed = 1500.0
buffersize = 128
target = 30000.0

min_energy = 0.6


def generate_signals(mics, pinger, target, speed, fs, buffersize):
    tdoa = compute_tdoa(pinger, mics, speed)
    signals = [
        generate_signal(buffersize, target, fs, time_offset=dt)
        for dt in tdoa
    ]
    return signals


def wait_for_ping(rate, signals, fs, target, max_period):
    global min_energy

    timeout = timedelta(seconds=max_period)

    # Wait until a signal of the target frequency is found.
    while not rospy.is_shutdown():
        start = datetime.now()
        while datetime.now() - start < timeout:
            rate.sleep()

            pings = [
                next(s)
                for s in signals
            ]

            for s in pings:
                energy = get_frequency_energy(s, fs, target)
                if energy >= min_energy:
                    min_energy = max(0.2, 0.95 * energy)
                    rospy.loginfo("Received %f, min energy now %f",
                                  energy, min_energy)
                    return pings

        min_energy = max(0.2, min_energy / 2)
        rospy.logwarn("Decreased min energy, now %f", min_energy)


def get_pings(signals, fs, target, buffersize, max_buffersize, max_period):
    # Set realistic rate.
    rate = rospy.Rate(fs / buffersize)

    # Wait for first instance of ping.
    pings = wait_for_ping(rate, signals, fs, target, max_period)

    # Fill the signal until max buffersize is reached.
    while len(pings[0]) < max_buffersize and not rospy.is_shutdown():
        for i, s in enumerate(pings):
            s.extend(next(signals[i]))

        rate.sleep()

    return pings


def to_signal_array_stamped(pings, frame):
    msg = SignalArrayStamped()

    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = frame

    msg.signals = [
        Signal(data=s)
        for s in pings
    ]

    return msg


if __name__ == "__main__":
    rospy.init_node("hydrophones_sim_audio")
    pub = rospy.Publisher("/hydrophones/audio", SignalArrayStamped,
                          queue_size=3)

    signals = generate_signals(mics, pinger, target, speed, fs, buffersize)
    while not rospy.is_shutdown():
        pings = get_pings(
            signals, fs, target, buffersize,
            max_buffersize=4096, max_period=2)
        msg = to_signal_array_stamped(pings, "odom")
        pub.publish(msg)
