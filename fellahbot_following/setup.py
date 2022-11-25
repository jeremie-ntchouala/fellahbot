#!/usr/bin/env python3
# from setuptools import setup
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['fellahbot_following'],
    scripts=['scripts'],
    package_dir={'': 'include'},
)

setup(**setup_args)
