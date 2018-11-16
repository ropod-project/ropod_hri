#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ropod_sound_vocalisation'],
    package_dir={'ropod_sound_vocalisation': 'ros/src/ropod_sound_vocalisation'}
)

setup(**d)
