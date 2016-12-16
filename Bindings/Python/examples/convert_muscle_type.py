# ----------------------------------------------------------------------- #
# The OpenSim API is a toolkit for musculoskeletal modeling and           #
# simulation. See http://opensim.stanford.edu and the NOTICE file         #
# for more information. OpenSim is developed at Stanford University       #
# and supported by the US National Institutes of Health (U54 GM072970,    #
# R24 HD065690) and by DARPA through the Warrior Web program.             #
#                                                                         #
# Copyright (c) 2005-2016 Stanford University and the Authors             #
# Author(s): Ayman Habib                                                  #
# Contributor(s):                                                         #
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

# convert_muscle_type.py
# Author: Ayman Habib
# ------------------------------------------------------------------------#
# ABSTRACT: This short piece of OpenSim python API example demonstrates   #
# how to load a model in the OpenSim python API, cycle through the        #
# the muscles in the model and replace those of type Thelen2003 with      #
# the more modern (true to publication) Millard2012AccelerationMuscle     #
# There are differencs between the two muscle models so the resulting     #
# model may need to be tuned manually.
# ------------------------------------------------------------------------#

import opensim as osim

import sys
# Helper function to copy Properties between two objects
def copy_common_muscle_properties(from_muscle, to_muscle):
    print ("Copying Properties for muscle:" +from_muscle.getName())
    # Common properties are in base class Muscle this includes
    # - max_isometric_force
    # - optimal_fiber_length
    # - tendon_slack_length
    # - pennation_angle_at_optimal
    # - max_contraction_velocity
    # - ignore_tendon_compliance
    # - ignore_activation_dynamics
    to_muscle.copyProperty_max_isometric_force(from_muscle)
    to_muscle.copyProperty_optimal_fiber_length(from_muscle)
    to_muscle.copyProperty_tendon_slack_length(from_muscle)
    to_muscle.copyProperty_pennation_angle_at_optimal(from_muscle)
    to_muscle.copyProperty_max_contraction_velocity(from_muscle)
    to_muscle.copyProperty_ignore_tendon_compliance(from_muscle)
    to_muscle.copyProperty_ignore_activation_dynamics(from_muscle)
    # Properties inherited from PathActuator
    to_muscle.set_GeometryPath(from_muscle.get_GeometryPath().clone())
    to_muscle.copyProperty_optimal_force(from_muscle)
    # Properties from ScalarActuator
    to_muscle.copyProperty_max_control(from_muscle)
    to_muscle.copyProperty_min_control(from_muscle)
    # Properties from Force
    to_muscle.copyProperty_appliesForce(from_muscle)
    # print (to_muscle.dump())
    
# Helper method to convert one Thelen muscle to Millard type
def convert_to_millard(one_thelen_muscle, one_millard_muscle):
    print ("Converting muscle:" +one_thelen_muscle.getName())
    one_millard_muscle.setName(one_thelen_muscle.getName())
    copy_common_muscle_properties(one_thelen_muscle, one_millard_muscle)
    one_millard_muscle.dump()

original_model = osim.Model("arm26.osim")
# Will be working with Properties only, although we may not need a state
# the traversal of the model objects requires initSystem
original_s = original_model.initSystem()
# Cycle through objects of type Thelen2003Muscle
thelen_muscles = original_model.getThelen2003MuscleList()

# Create ForceSet to contain the new muscles
millard_muscles = osim.ForceSet()
# Ownership will be transfered to the converted_model eventually
millard_muscles.setMemoryOwner(False)


for thelen_musc in thelen_muscles:
    print ("Processing muscle:"+thelen_musc.getName())
    millard_musc = osim.Millard2012AccelerationMuscle()
    convert_to_millard(thelen_musc, millard_musc)
    millard_muscles.cloneAndAppend(millard_musc)
    

# Now clone the model and remove all Thelen muscles
convert_model = original_model.clone();
# remove Thelen muscles 
convert_model_forces = convert_model.getForceSet()
convert_model_forces.dump()
for thelen_musc in thelen_muscles:
    index = convert_model_forces.getIndex(thelen_musc.getName())
    if (index != -1):
        convert_model_forces.remove(index)
        
# Now insert Millard muscles
for i in range(millard_muscles.getSize()):
    convert_model.addForce(millard_muscles.get(i))

# ---------------------------------------------------------------------------
# Print/save model file
# ---------------------------------------------------------------------------
# print converted model
convert_model.printToXML("convertedModel.osim")
