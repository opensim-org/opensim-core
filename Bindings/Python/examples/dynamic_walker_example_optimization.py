# ----------------------------------------------------------------------- #
# The OpenSim API is a toolkit for musculoskeletal modeling and           #
# simulation. See http://opensim.stanford.edu and the NOTICE file         #
# for more information. OpenSim is developed at Stanford University       #
# and supported by the US National Institutes of Health (U54 GM072970,    #
# R24 HD065690) and by DARPA through the Warrior Web program.             #
#                                                                         #
# Copyright (c) 2005-2019 Stanford University and the Authors             #
# Author(s): Thomas Uchida, Akshaykumar Patel, James Dunne                #
# Contributor(s): Andrew Miller, Jeff Reinbolt, Ajay Seth, Dan Jacobs,    #
#                 Chris Dembia, Ayman Habib, Carmichael Ong               #
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

# This example demonstrates how to run an optimization in Python using the
# cma package (see References, below). The model is a dynamic walker (see
# the analogous Matlab example files for details); the optimizer adjusts the
# initial forward velocity of the pelvis and the initial angles and angular
# velocities of the knees and hips to maximize travel distance in 10 seconds.
# Note that the OpenSim model 'dynamic_walker_example_model.osim' must be in
# the same folder as this script and the cma package must have been installed.
# As configured, the optimizer takes about 3 minutes to run on a standard
# laptop (Intel i7, Windows 10, Python 2.7). For demonstration purposes only.
#
# References for cma package:
# [1] Project page -- https://pypi.org/project/cma/
# [2] Implementation -- https://github.com/CMA-ES/pycma
# [3] Documentation -- http://cma.gforge.inria.fr/apidocs-pycma/cma.html

import opensim as osim
import time
import cma


# OBJECTIVE FUNCTION
# Runs a forward simulation using the initial conditions specified in the
# candidate solution vector (candsol) and computes the corresponding
# objective function value (i.e. the final location of the pelvis).
def walker_simulation_objective_function(candsol):
    global model, initial_state, all_distances, all_candsols
    
    # Set the initial hip and knee angles.
    initial_state.updQ()[3] = candsol[0]    # left hip
    initial_state.updQ()[4] = candsol[1]    # right hip
    initial_state.updQ()[5] = candsol[2]    # left knee
    initial_state.updQ()[6] = candsol[3]    # right knee
    
    # Set the initial forward velocity of the pelvis.
    vx0 = candsol[4]    # needed to compute the objective function, below
    initial_state.updU()[1] = vx0
    
    # Set the initial hip and knee angular velocities.
    initial_state.updU()[3] = candsol[5]    # left hip
    initial_state.updU()[4] = candsol[6]    # right hip
    initial_state.updU()[5] = candsol[7]    # left knee
    initial_state.updU()[6] = candsol[8]    # right knee
    
    # Simulate.
    manager = osim.Manager(model)
    manager.initialize(initial_state)
    manager.integrate(10.0)
    
    # Get the final location of the pelvis in the X direction.
    st = manager.getStatesTable()
    dc = st.getDependentColumn('/jointset/PelvisToPlatform/Pelvis_tx/value')
    x  = dc[ dc.nrow()-1 ]
    # Store the candidate solution and the distance traveled.
    all_candsols.append(candsol)
    all_distances.append(x)
    print('Distance traveled: %f meters' % (x))
    
    # To maximize distance, minimize its negative. Also penalize candidate
    # solutions that increase the initial pelvis velocity beyond 2 m/s (to
    # avoid simply launching the model forward). This could also be done by
    # adding a hard constraint.
    k = 10.0
    penalty1 = k*(vx0**2.0) if vx0 < 0 else 0.0         # lower bound: 0 m/s
    penalty2 = k*((vx0-2.0)**2.0) if vx0 > 2 else 0.0   # upper bound: 2 m/s
    J = -x + penalty1 + penalty2
    return (J)

# MAIN
# Perform an optimization using cma with the above objective function. The
# final model will be saved as 'dynamic_walker_example_model_optimized.osim'.
global model, initial_state, all_distances, all_candsols
all_distances = []
all_candsols  = []

# Load OpenSim model.
model = osim.Model('dynamic_walker_example_model.osim')

# Create the underlying computational system. Note that we reuse the State
# in the objective function to avoid the high computational cost of calling
# initSystem() before every simulation.
initial_state = model.initSystem()

# Create candidate solution vector. The (arbitrary) initial guess for the
# initial forward velocity of the pelvis is 0.1. If the optimizer is working
# correctly, it should increase this value to 2.0 (the upper bound); see the
# penalty calculation in the objective function.
candsol = []
candsol.append(model.getCoordinateSet().get('LHip_rz').getDefaultValue())
candsol.append(model.getCoordinateSet().get('RHip_rz').getDefaultValue())
candsol.append(model.getCoordinateSet().get('LKnee_rz').getDefaultValue())
candsol.append(model.getCoordinateSet().get('RKnee_rz').getDefaultValue())
candsol.append(0.1)
candsol.append(model.getCoordinateSet().get('LHip_rz').getDefaultSpeedValue())
candsol.append(model.getCoordinateSet().get('RHip_rz').getDefaultSpeedValue())
candsol.append(model.getCoordinateSet().get('LKnee_rz').getDefaultSpeedValue())
candsol.append(model.getCoordinateSet().get('RKnee_rz').getDefaultSpeedValue())

# Optimize.
t_start = time.time()
# For a description of arguments to fmin(), run cma.CMAOptions() or see
# http://cma.gforge.inria.fr/apidocs-pycma/cma.evolution_strategy.html#fmin
result = cma.fmin(walker_simulation_objective_function, candsol, 0.5,
                  options = {'popsize':20, 'tolfun':1e-3, 'tolx':1e-3,
                             'maxfevals':100})
t_elapsed = time.time() - t_start
print('Elapsed time: %f seconds' % (t_elapsed))

# Find the best solution.
max_distance = max(all_distances)
print('Best distance: %f meters' % (max_distance))
idx = all_distances.index(max_distance)
bestsol = all_candsols[idx]
print(bestsol)

# Assign best solution to model and save.
model.getCoordinateSet().get('LHip_rz').setDefaultValue(bestsol[0])
model.getCoordinateSet().get('RHip_rz').setDefaultValue(bestsol[1])
model.getCoordinateSet().get('LKnee_rz').setDefaultValue(bestsol[2])
model.getCoordinateSet().get('RKnee_rz').setDefaultValue(bestsol[3])
model.getCoordinateSet().get('Pelvis_tx').setDefaultSpeedValue(bestsol[4])
model.getCoordinateSet().get('LHip_rz').setDefaultSpeedValue(bestsol[5])
model.getCoordinateSet().get('RHip_rz').setDefaultSpeedValue(bestsol[6])
model.getCoordinateSet().get('LKnee_rz').setDefaultSpeedValue(bestsol[7])
model.getCoordinateSet().get('RKnee_rz').setDefaultSpeedValue(bestsol[8])
model.printToXML('dynamic_walker_example_model_optimized.osim')
