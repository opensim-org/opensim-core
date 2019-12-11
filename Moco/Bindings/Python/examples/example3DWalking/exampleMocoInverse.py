# -------------------------------------------------------------------------- #
# OpenSim Moco: exampleMocoInverse.py                                        #
# -------------------------------------------------------------------------- #
# Copyright (c) 2019 Stanford University and the Authors                     #
#                                                                            #
# Author(s): Christopher Dembia                                              #
#                                                                            #
# Licensed under the Apache License, Version 2.0 (the "License") you may     #
# not use this file except in compliance with the License. You may obtain a  #
# copy of the License at http://www.apache.org/licenses/LICENSE-2.0          #
#                                                                            #
# Unless required by applicable law or agreed to in writing, software        #
# distributed under the License is distributed on an "AS IS" BASIS,          #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
# See the License for the specific language governing permissions and        #
# limitations under the License.                                             #
# -------------------------------------------------------------------------- #

# This example shows how to use the MocoInverse tool to exactly prescribe a
# motion and estimate muscle behavior for walking.
# This problem solves in about 5 minutes.
#
# See the README.txt next to this file for more information.

import opensim as osim

# Construct the MocoInverse tool.
inverse = osim.MocoInverse()

# Construct a ModelProcessor and set it on the tool. The default
# muscles in the model are replaced with optimization-friendly
# DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
# parameters.
modelProcessor = osim.ModelProcessor('subject_walk_armless.osim')
modelProcessor.append(osim.ModOpAddExternalLoads('grf_walk.xml'))
modelProcessor.append(osim.ModOpIgnoreTendonCompliance())
modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
# Only valid for DeGrooteFregly2016Muscles.
modelProcessor.append(osim.ModOpIgnorePassiveFiberForcesDGF())
# Only valid for DeGrooteFregly2016Muscles.
modelProcessor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
modelProcessor.append(osim.ModOpAddReserves(1.0))
inverse.setModel(modelProcessor)

# Construct a TableProcessor of the coordinate data and pass it to the
# inverse tool. TableProcessors can be used in the same way as
# ModelProcessors by appending TableOperators to modify the base table.
# A TableProcessor with no operators, as we have here, simply returns the
# base table.
inverse.setKinematics(osim.TableProcessor('coordinates.sto'))

# Initial time, final time, and mesh interval.
inverse.set_initial_time(0.81)
inverse.set_final_time(1.79)
inverse.set_mesh_interval(0.02)

# By default, Moco gives an error if the kinematics contains extra columns.
# Here, we tell Moco to allow (and ignore) those extra columns.
inverse.set_kinematics_allow_extra_columns(True)

# Solve the problem and write the solution to a Storage file.
solution = inverse.solve()
solution.getMocoSolution().write('example3DWalking_MocoInverse_solution.sto')

# Generate a PDF with plots for the solution trajectory.
model = modelProcessor.process()
report = osim.Report(model, 'example3DWalking_MocoInverse_solution.sto',
                     bilateral=True)
# The PDF is saved to the working directory.
report.generate()
