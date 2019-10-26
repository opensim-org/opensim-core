# ----------------------------------------------------------------------- #
# The OpenSim API is a toolkit for musculoskeletal modeling and           #
# simulation. See http://opensim.stanford.edu and the NOTICE file         #
# for more information. OpenSim is developed at Stanford University       #
# and supported by the US National Institutes of Health (U54 GM072970,    #
# R24 HD065690) and by DARPA through the Warrior Web program.             #
#                                                                         #
# Copyright (c) 2005-2019 Stanford University and the Authors             #
# Author(s): Thomas Uchida, Akshaykumar Patel                             #
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

# This example demonstrates how to extend the Vec3 class so that two Vec3s
# can be added in Python using `+` and the result is a Vec3. An analogous
# strategy can be used for other operators.
#
# Reference:
# https://stackoverflow.com/questions/50599045/
# python-replacing-a-function-within-a-class-of-a-module

import opensim as osim

# Define the function that will be used when the `+` operator is called
# with two Vec3s.
def myVec3Add(self,v):
    newvec = osim.Vec3(self[0]+v[0], self[1]+v[1], self[2]+v[2])
    return newvec

# Assign this function to `operator+` in the existing Vec3 class.
osim.Vec3.__add__ = myVec3Add

# Test.
a = osim.Vec3(1,2,3)
b = osim.Vec3(4,5,6)
c = a+b
print(c)
print(type(c))
