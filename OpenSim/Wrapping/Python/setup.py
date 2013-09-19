#!/usr/bin/env python

import os

from setuptools import setup, find_packages

if os.name == 'posix':
    # Linux, etc.
    lib_name = '_opensim.so'
elif os.name == 'nt':
    # Windows.
    lib_name = '_opensim.pyd'

setup(name='opensim',
      version='3.1',
      description='OpenSim Simulation Framework',
      author='OpenSim Team',
      author_email='ahabib@stanford.edu',
      url='http://opensim.stanford.edu/',
      license='Apache 2.0',
      packages=find_packages(),
      data_files=[('opensim', ['opensim/%s' % lib_name])],
      classifiers=[
          'Intended Audience :: Science/Research',
          'Operating System :: OS Independent',
          'Programming Language :: Python :: 2.7',
          'Topic :: Scientific/Engineering :: Physics',
          ],
      )
