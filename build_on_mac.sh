#!/bin/bash
# Please see README.md for instructions to install 
# the dependencies for this script.
# This is a Unix Shell script that performs the following steps:
# 1. Obtain/build Moco's dependencies.
# 2. Build Moco.
# 3. Test Moco.
# 4. Install Moco.
# The script will make multiple directories adjacent to the directory
# containing this script.

git submodule update --init
cd ..
mkdir moco_dependencies_build
cd moco_dependencies_build
cmake ../opensim-moco/dependencies
make -j8
cd ..
mkdir build
cd build
cmake ../opensim-moco
make -j8
ctest -j8
make -j8 install
