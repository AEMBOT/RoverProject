#!/usr/bin/env python

# Setup the current package so auxilary code can be used and not be executable

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['drive_train'],
     package_dir={'': 'src'}
)

setup(**setup_args)