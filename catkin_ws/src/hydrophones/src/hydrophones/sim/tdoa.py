# -*- coding: utf-8 -*-

"""Time difference of arrival simulator."""

import numpy as np

__author__ = "Anass Al-Wohoush"


def compute_tdoa(pinger, mics, speed):
    """Computes theoretical time difference of arrival based on position of
    pinger and microphones and speed of sound.

    Args:
        pinger: Pinger object.
        mics: List of Mics. The first element must be the origin microphone.
        speed: Speed of sound in the medium in m/s.

    Returns:
        List of time difference of arrivals in seconds. The size of the list is
        the number of mics.
    """
    # Compute theoretical travel time for each mic using Pythagorean theorem.
    dt = []
    for mic in mics:
        dx, dy, dz = pinger.x - mic.x, pinger.y - mic.y, pinger.z - mic.z
        d = np.linalg.norm((dx, dy, dz))
        dt.append(d / speed)

    # Compute travel time difference between each mic and the origin mic.
    tdoa = [
        (t - dt[0])
        for t in dt
    ]

    return tdoa
