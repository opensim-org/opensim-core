# ----------------------------------------------------------------------- #
# The OpenSim API is a toolkit for musculoskeletal modeling and           #
# simulation. See http://opensim.stanford.edu and the NOTICE file         #
# for more information. OpenSim is developed at Stanford University       #
# and supported by the US National Institutes of Health (U54 GM072970,    #
# R24 HD065690) and by DARPA through the Warrior Web program.             #
#                                                                         #
# Copyright (c) 2005-2020 Stanford University and the Authors             #
# Author(s): Christopher Dembia                                           #
#                                                                         #
# Licensed under the Apache License, Version 2.0 (the "License");         #
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

# This example shows how to convert NumPy arrays to and from OpenSim's Vector
# and Matrix classes.

import numpy as np

# Create an OpenSim Vec3 from a NumPy array.
npArray = np.array([5, 3, 6])
osimVec3 = osim.Vec3.createFromMat(npArray)
print(osimVec3)
# Convert the OpenSim Vec3 back to a NumPy array.
npArray1 = osimVec3.to_numpy()
print(npArray1)

import opensim as osim

# Create an OpenSim Vector from a NumPy array.
npArray = np.array([5, 3, 6, 2, 9])
osimVector = osim.Vector.createFromMat(npArray)
print(osimVector)
# Convert the OpenSim Vector back to a NumPy array.
npArray2 = osimVector.to_numpy()
print(npArray2)

# Same for RowVector.
osimRowVector = osim.RowVector.createFromMat(npArray)
print(osimRowVector)
npArray3 = osimRowVector.to_numpy()
print(npArray3)

# Same for Matrix.
npArray2D = np.array([[5, 3], [3, 6], [8, 1]])
osimMatrix = osim.Matrix.createFromMat(npArray2D)
print(osimMatrix)
npArray4 = osimMatrix.to_numpy()
print(npArray4)

