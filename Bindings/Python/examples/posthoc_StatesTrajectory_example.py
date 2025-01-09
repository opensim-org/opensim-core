# ----------------------------------------------------------------------- #
# The OpenSim API is a toolkit for musculoskeletal modeling and           #
# simulation. See http://opensim.stanford.edu and the NOTICE file         #
# for more information. OpenSim is developed at Stanford University       #
# and supported by the US National Institutes of Health (U54 GM072970,    #
# R24 HD065690) and by DARPA through the Warrior Web program.             #
#                                                                         #
# Copyright (c) 2005-2018 Stanford University and the Authors             #
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

# This example shows how to use the StatesTrajectory to analyze a simulation
# (computing joint reaction forces).

import opensim as osim
import math

model = osim.Model()
model.setName('model')

# Create a pendulum model.
# ------------------------
# In the default pose, the system is a mass hanging 1 meter down from a hinge.
body = osim.Body('body', 1.0, osim.Vec3(0), osim.Inertia(1))
model.addComponent(body)

joint = osim.PinJoint('joint',
        model.getGround(), osim.Vec3(0), osim.Vec3(0), 
        body, osim.Vec3(0, 1.0, 0), osim.Vec3(0))
joint.updCoordinate().setName('q')
model.addComponent(joint)

# This reporter will collect states throughout the simulation at intervals of
# 0.05 seconds.
reporter = osim.StatesTrajectoryReporter()
reporter.setName('reporter')
reporter.set_report_time_interval(0.05)
model.addComponent(reporter)

# Simulate.
# ---------
state = model.initSystem()

# The initial position is that the pendulum is rotated 45 degrees
# counter-clockwise from its default pose.
model.setStateVariableValue(state, 'joint/q/value', 0.25 * math.pi)

manager = osim.Manager(model, state)
manager.integrate(1.0)

# Analyze the simulation.
# -----------------------
# Retrieve the StatesTrajectory from the reporter.
statesTraj = reporter.getStates()

for itime in range(statesTraj.getSize()):
    state = statesTraj[itime]
    time = state.getTime()
    q = model.getStateVariableValue(state, 'joint/q/value')

    # Calculating joint reactions requires realizing to Acceleration.
    model.realizeAcceleration(state)
    # This returns a SpatialVec, which is two Vec3's ([0]: moment, [1]: force).
    reaction = joint.calcReactionOnParentExpressedInGround(state)
    force = reaction.get(1)
    force_mag = math.sqrt(force[0]**2 + force[1]**2 + force[2]**2)
    print('time: %f s.  q: %f rad.  reaction force magnitude: %f N.' % (
        time, q, force_mag))

    # The reaction could also be accessed as an output:
    # abstractOutput = joint.getOutput('reaction_on_parent')
    # output = osim.OutputSpatialVec.safeDownCast(abstractOutput)
    # outputValue = output.getValue(s)

# Alternately, we could load a StatesTrajectory from a TimeSeriesTable file
# using StatesTrajectory.createFromStatesTable().
statesTable = manager.getStatesTable()
statesTraj2 = osim.StatesTrajectory.createFromStatesTable(model, statesTable)

for itime in range(statesTraj2.getSize()):
    state = statesTraj2[itime]
    time = state.getTime()
    u = model.getStateVariableValue(state, 'joint/q/speed')
    print('time: %f s.  u: %f rad/s.' % (time, u))
