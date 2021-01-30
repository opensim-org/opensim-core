# -------------------------------------------------------------------------- #
# OpenSim Moco: exampleOptimizeMass.py                                       #
# -------------------------------------------------------------------------- #
# Copyright (c) 2019 Stanford University and the Authors                     #
#                                                                            #
# Author(s): Noah Gordon, Jennifer Yong                                      #
#                                                                            #
# Licensed under the Apache License, Version 2.0 (the "License"); you may    #
# not use this file except in compliance with the License. You may obtain a  #
# copy of the License at http://www.apache.org/licenses/LICENSE-2.0          #
#                                                                            #
# Unless required by applicable law or agreed to in writing, software        #
# distributed under the License is distributed on an "AS IS" BASIS,          #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
# See the License for the specific language governing permissions and        #
# limitations under the License.                                             #
# -------------------------------------------------------------------------- #

# Optimize the mass of a simple harmonic oscillator such that it follows the 
# correct trajectory specified by the state bounds and the MocoMarkerFinalGoal.

import os
import opensim as osim
import numpy as np

# Setting variables
stiffness = 100.
mass = 5.
finalTime = np.pi * np.sqrt(mass / stiffness)

# Defining the model
model = osim.Model()
model.setName('oscillator')
model.set_gravity(osim.Vec3(0, 0, 0))
body = osim.Body('body', np.multiply(0.5, mass), osim.Vec3(0), osim.Inertia(0))
model.addComponent(body)

# Adding a marker to body in the model
marker = osim.Marker('marker', body, osim.Vec3(0))
model.addMarker(marker)

# Allows translation along x.
joint = osim.SliderJoint('slider', model.getGround(), body)
coord = joint.updCoordinate()
coord.setName('position')
model.addComponent(joint)

# Adds the spring component
spring = osim.SpringGeneralizedForce()
spring.set_coordinate('position')
spring.setRestLength(0.)
spring.setStiffness(stiffness)
spring.setViscosity(0.)
model.addComponent(spring)


# Create MocoStudy.
# ================
moco = osim.MocoStudy()
moco.setName('oscillator_spring_stiffness')


# Define the optimal control problem.
# ===================================
problem = moco.updProblem()

# Model (dynamics).
# -----------------
problem.setModel(model)

# Bounds.
# -------
# Initial time must be 0, final time is finalTime.
problem.setTimeBounds(osim.MocoInitialBounds(0.),
                      osim.MocoFinalBounds(finalTime))

# Position must be within [-5, 5] throughout the motion.
# Initial position must be -0.5, final position must be within [0.25, 0.75].
problem.setStateInfo('/slider/position/value', osim.MocoBounds(-5., 5.),
                     osim.MocoInitialBounds(-0.5),
                     osim.MocoFinalBounds(0.25, 0.75))

# Speed must be within [-20, 20] throughout the motion.
# Initial and final speed must be 0. Use compact syntax.
problem.setStateInfo('/slider/position/speed', [-20, 20], [0], [0])

# Add Parameter. The default initial guess for a parameter is the midpoint of
# its bounds, *not* the value of a property in the model.
problem.addParameter(osim.MocoParameter('oscillator_mass', 'body', 'mass',
                                        osim.MocoBounds(0, 10)))

# Cost.
# -----
endpointCost = osim.MocoMarkerFinalGoal()
endpointCost.setPointName('/markerset/marker')
endpointCost.setReferenceLocation(osim.Vec3(0.5, 0, 0))
problem.addGoal(endpointCost)


# Configure the solver.
# =====================
solver = moco.initTropterSolver()

# Now that we've finished setting up the study, print it to a file.
moco.printToXML('optimize_mass.omoco')

# Solve the problem.
# ==================
solution = moco.solve()
solution.write('optimize_mass_solution.sto')
