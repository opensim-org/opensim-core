# ----------------------------------------------------------------------- #
# The OpenSim API is a toolkit for musculoskeletal modeling and           #
# simulation. See http://opensim.stanford.edu and the NOTICE file         #
# for more information. OpenSim is developed at Stanford University       #
# and supported by the US National Institutes of Health (U54 GM072970,    #
# R24 HD065690) and by DARPA through the Warrior Web program.             #
#                                                                         #
# Copyright (c) 2005-2019 Stanford University and the Authors             #
# Author(s): James Dunne                                                  #
#                                                                         #
# Licensed under the Apache License, Version 2.0 (the "License")         #
# you may not use this file except in compliance with the License.        #
# You may obtain a copy of the License at                                 #
# http://www.apache.org/licenses/LICENSE-2.0.                             #
#                                                                         #
# Unless required by applicable law or agreed to in writing, software     #
# distributed under the License is distributed on an "AS IS" BASIS,       #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         #
# implied. See the License for the specific language governing            #
# permissions and limitations under the License.                          #
# ----------------------------------------------------------------------- #

# Example code for reading, and converting, XSENS IMU sensor data to
# OpenSense friendly format.
# Run this script from the OpenSenseExampleFiles directory. 

# Import the opensim libraries
import opensim as osim

# Build an Xsens Settings Object. 
# Instantiate the Reader Settings Class
xsensSettings = osim.XsensDataReaderSettings('myIMUMappings.xml')
# Instantiate an XsensDataReader
xsens = osim.XsensDataReader(xsensSettings)
# Read in seprate tables of data from the specified IMU file(s)
tables = xsens.read('IMUData/')
# get the trial name from the settings
trial = xsensSettings.get_trial_prefix()
# Get Orientation Data as quaternions
quatTable = xsens.getOrientationsTable(tables)
# Write to file
osim.STOFileAdapterQuaternion.write(quatTable,  trial + '_orientations.sto')
# Get Acceleration Data
accelTable = xsens.getLinearAccelerationsTable(tables)
# Write to file
osim.STOFileAdapterVec3.write(accelTable, trial + '_linearAccelerations.sto')
# Get Magnetic (North) Heading Data
magTable = xsens.getMagneticHeadingTable(tables)
# Write to file
osim.STOFileAdapterVec3.write(magTable, trial + '_magneticNorthHeadings.sto')
# Get Angular Velocity Data
angVelTable = xsens.getAngularVelocityTable(tables)
# Write to file
osim.STOFileAdapterVec3.write(angVelTable, trial + '_angularVelocities.sto')