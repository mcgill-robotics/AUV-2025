#!/usr/bin/env python
"""Teledyne Navigator DVL ROS package setup."""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["teledyne_navigator"], package_dir={"": "src"})

setup(**d)
