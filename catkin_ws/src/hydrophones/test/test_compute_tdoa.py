#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Tests hydrophones TDOA simulation."""

import rosunit
import numpy as np
from hydrophones import Mic
from unittest import TestCase
from hydrophones.sim import Pinger, compute_tdoa

__author__ = "Anass Al-Wohoush"


class TestComputeTDOA(TestCase):

    """Verifies sim.tdoa.compute_tdoa functions as expected."""

    def test_compute_tdoa(self):
        """Verifies simulated TDOAs are correct."""
        mics = [
            Mic((0.00, 0.00, 0.00)),
            Mic((0.00, 0.01, 0.00)),
            Mic((0.01, 0.01, 0.00)),
            Mic((0.01, 0.00, 0.00))
        ]
        pinger = Pinger((100.00, 20.00, 4.00))
        speed = 1500.00
        tdoa = compute_tdoa(pinger, mics, speed)
        expected_tdoa = [0.0, -1.306e-06, -7.838e-06, -6.532e-06]

        for i in range(len(mics)):
            self.assertAlmostEqual(expected_tdoa[i], tdoa[i])


if __name__ == "__main__":
    np.random.seed(1)
    rosunit.unitrun("test_hydrophones", "test_compute_tdoa", TestComputeTDOA)
