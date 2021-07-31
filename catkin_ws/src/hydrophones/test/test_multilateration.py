#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Tests hydrophones TDOA simulation."""

import rosunit
import numpy as np
from unittest import TestCase
from hydrophones import Mic
from hydrophones.multilateration import solve
from hydrophones.sim import Pinger, compute_tdoa
from hydrophones.exceptions import CouldNotBeSolvedError

__author__ = "Anass Al-Wohoush"


class TestMultilateration(TestCase):

    """Verifies multilateration."""

    def setUp(self):
        """Sets up test."""
        self.mics = [
            Mic((0.00, 0.00, 0.00)),
            Mic((0.00, 0.01, 0.00)),
            Mic((0.01, 0.01, 0.00)),
            Mic((0.01, 0.00, 0.00))
        ]
        self.speed = 1500.00

    def test_far_away(self):
        """Multilateration solver works if pinger is far away."""
        positions = [
            (100.00, 20.00, 4.00),
            (-100.00, 20.00, 4.00),
            (100.00, -20.00, 4.00),
            (-100.00, -20.00, 4.00)
        ]
        for x, y, z in positions:
            pinger = Pinger((x, y, z))
            tdoa = compute_tdoa(pinger, self.mics, self.speed)
            x, y = solve(self.mics, tdoa, self.speed)

            self.assertAlmostEqual(x, pinger.x, delta=2.00)
            self.assertAlmostEqual(y, pinger.y, delta=2.00)

    def test_midrange(self):
        """Multilateration solver works if pinger is midrange."""
        positions = [
            (10.00, 20.00, 4.00),
            (-10.00, 20.00, 4.00),
            (10.00, -20.00, 4.00),
            (-10.00, -20.00, 4.00)
        ]
        for x, y, z in positions:
            pinger = Pinger((x, y, z))
            tdoa = compute_tdoa(pinger, self.mics, self.speed)
            try:
                x, y = solve(self.mics, tdoa, self.speed)
            except CouldNotBeSolvedError as e:
                self.fail("Could not solve: {}".format(e))
                continue

            self.assertAlmostEqual(x, pinger.x, delta=1.00)
            self.assertAlmostEqual(y, pinger.y, delta=1.00)

    def test_nearby(self):
        """Multilateration solver works if pinger is nearby."""
        positions = [
            (1.00, 2.00, 4.00),
            (-1.00, 2.00, 4.00),
            (1.00, -2.00, 4.00),
            (-1.00, -2.00, 4.00)
        ]
        for x, y, z in positions:
            pinger = Pinger((x, y, z))
            tdoa = compute_tdoa(pinger, self.mics, self.speed)
            try:
                x, y = solve(self.mics, tdoa, self.speed)
            except CouldNotBeSolvedError as e:
                self.fail("Could not solve: {}".format(e))
                continue

            self.assertAlmostEqual(x, pinger.x, delta=0.05)
            self.assertAlmostEqual(y, pinger.y, delta=0.05)

    def test_underneath(self):
        """Multilateration solver works if above pinger."""
        positions = [
            (0.01, 0.005, 4.00),
            (-0.01, 0.005, 4.00),
            (0.01, -0.005, 4.00),
            (-0.01, -0.005, 4.00)
        ]
        for x, y, z in positions:
            pinger = Pinger((x, y, z))
            tdoa = compute_tdoa(pinger, self.mics, self.speed)
            print tdoa
            try:
                x, y = solve(self.mics, tdoa, self.speed)
            except CouldNotBeSolvedError as e:
                self.fail("Could not solve: {}".format(e))
                continue

            self.assertAlmostEqual(x, pinger.x, delta=0.05)
            self.assertAlmostEqual(y, pinger.y, delta=0.05)


if __name__ == "__main__":
    np.random.seed(1)
    rosunit.unitrun("test_hydrophones", "test_multilateration",
                    TestMultilateration)
