# -*- coding: utf-8 -*-

"""Through multilateration, solves for location of pinger given TDOAs.

This method requires at least three TDOAs (i.e. four hydrophones) in order to
work, and works as is given more. Accuracy increases as the number of TDOAs
increases.

Unfortunately, the accuracy of the estimated distance to the pinger is heavily
dependent on the accuracy of the TDOAs, hence can be very unustable. The
estimated heading however is very accurate and is sufficient to solve the task.

The algorithm is adapted to two-dimensional space from
http://en.wikipedia.org/wiki/Multilateration
"""

import numpy as np
from random import random
from .exceptions import CouldNotBeSolvedError

__author__ = "Anass Al-Wohoush"


def solve(mics, tdoa, speed):
    """Solves for 2D coordinates of pinger given list of time difference of
    arrivals.

    Args:
        mics: List of Mics. The first element must be the origin microphone.
        tdoa: List of time difference of arrivals in seconds.
        speed: Speed of sound in the medium in m/s.

    Returns:
        Estimated (X, Y) coordinates of pinger in meters.

    Raises:
        CouldNotBeSolvedError: Solution could not be found.
    """
    NUMBER_OF_MICS = len(mics)

    # Force raise Error instead of printing RuntimeWarning.
    np.seterr(all="raise")

    # Add picosecond random noise to decrease the chances of having a singular
    # matrix.
    tdoa = [
        dt + random() * 1e-12
        for dt in tdoa
    ]

    try:
        # Construct vectors.
        # All vectors were multiplied by t[1] * t[0] in order to avoid division
        # by zero.
        # A[m] = 2 * ((x[m] * t[1]) - (x[1] * t[m])) / v
        # B[m] = 2 * ((y[m] * t[1]) - (y[1] * t[m])) / v
        # C[m] = t[m] * t[1] * v * (t[m] - t[1])
        #      - t[1] * (x[m]^2 + y[m]^2) / v
        #      + t[m] * (x[1]^2 + y[1]^2) / v
        A = [
            (2 / speed) * ((mics[i].x * tdoa[1]) - (mics[1].x * tdoa[i]))
            for i in range(2, NUMBER_OF_MICS)
        ]

        B = [
            (2 / speed) * ((mics[i].y * tdoa[1]) - (mics[1].y * tdoa[i]))
            for i in range(2, NUMBER_OF_MICS)
        ]

        C = [
            tdoa[i] * tdoa[1] * speed * (tdoa[i] - tdoa[1]) -
            (mics[i].x ** 2 + mics[i].y ** 2) * tdoa[1] / speed +
            (mics[1].x ** 2 + mics[1].y ** 2) * tdoa[i] / speed
            for i in range(2, NUMBER_OF_MICS)
        ]

        # Solve for (x, y) by QR.
        # 0 = A[m]x + B[m]y + C[m]
        try:
            (x, y) = -np.linalg.solve(np.transpose([A, B]), C)
        except np.linalg.LinAlgError as e:
            # Matrix is either singular or impossible to solve.
            raise CouldNotBeSolvedError(e)
    except (FloatingPointError, ZeroDivisionError) as e:
        # Overflow, underflow, division by zero or invalid error.
        raise CouldNotBeSolvedError(e)

    return (x, y)
