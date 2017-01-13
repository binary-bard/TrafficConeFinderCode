#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

args = generate_distutils_setup(
    packages=['cone_finder'],
    package_dir={'': 'src'},
    scripts=['scripts']
    )

setup(**args)
