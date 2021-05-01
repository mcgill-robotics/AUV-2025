# -*- coding: utf-8 -*-

"""Microphone model."""

__author__ = "Anass Al-Wohoush"


class Mic(object):

    """Microphone object."""

    def __init__(self, (x, y, z)):
        """Constructs Mic instance.

        Args:
            (x, y, z): 3D coordinates of mic relative to origin mic in meters.
        """
        self.x = x
        self.y = y
        self.z = z
