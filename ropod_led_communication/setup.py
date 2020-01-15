#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ropod_led_communication'],
    package_dir={'ropod_led_communication': 'ros/src/ropod_led_communication'}
)

setup(**d)
