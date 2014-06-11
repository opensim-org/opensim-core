#!/usr/bin/env python

import os

from setuptools import setup

setup(name='opensim',
      version='3.2',
      description='OpenSim Simulation Framework',
      author='OpenSim Team',
      author_email='ahabib@stanford.edu',
      url='http://opensim.stanford.edu/',
      license='Apache 2.0',
      packages=['opensim'],
      package_data={'opensim': ['_opensim.*']},
      include_package_data=True,
      classifiers=[
          'Intended Audience :: Science/Research',
          'Operating System :: OS Independent',
          'Programming Language :: Python :: 2.7',
          'Topic :: Scientific/Engineering :: Physics',
          ],
      )
