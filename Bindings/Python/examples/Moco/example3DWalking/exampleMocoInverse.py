# -------------------------------------------------------------------------- #
# OpenSim Moco: exampleMocoInverse.py                                        #
# -------------------------------------------------------------------------- #
# Copyright (c) 2020 Stanford University and the Authors                     #
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
# motion and estimate muscle behavior for walking. The first example does not
# rely on electromyography data, while the second example penalizes deviation
# from electromyography data for a subset of muscles.
#
# Both examples use the Python utility osim.report to automatically generate a
# PDF that includes the trajectories of all states and controls in the solution.
# This utility requires a Python environment with Matplotlib and NumPy installed.
#
# See the README.txt next to this file for more information.

import opensim as osim

def solveMocoInverse():

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
    report = osim.report.Report(model,
                                'example3DWalking_MocoInverse_solution.sto',
                                bilateral=True)
    # The PDF is saved to the working directory.
    report.generate()

def solveMocoInverseWithEMG():

    # This initial block of code is identical to the code above.
    inverse = osim.MocoInverse()
    modelProcessor = osim.ModelProcessor('subject_walk_armless.osim')
    modelProcessor.append(osim.ModOpAddExternalLoads('grf_walk.xml'))
    modelProcessor.append(osim.ModOpIgnoreTendonCompliance())
    modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
    modelProcessor.append(osim.ModOpIgnorePassiveFiberForcesDGF())
    modelProcessor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
    modelProcessor.append(osim.ModOpAddReserves(1.0))
    inverse.setModel(modelProcessor)
    inverse.setKinematics(osim.TableProcessor('coordinates.sto'))
    inverse.set_initial_time(0.81)
    inverse.set_final_time(1.79)
    inverse.set_mesh_interval(0.02)
    inverse.set_kinematics_allow_extra_columns(True)

    study = inverse.initialize()
    problem = study.updProblem()

    # Add electromyography tracking.
    emgTracking = osim.MocoControlTrackingGoal('emg_tracking')
    emgTracking.setWeight(50.0)
    # Each column in electromyography.sto is normalized so the maximum value in
    # each column is 1.0.
    controlsRef = osim.TimeSeriesTable('electromyography.sto')
    # Scale the tracked muscle activity based on peak levels from
    # "Gait Analysis: Normal and Pathological Function" by
    # Perry and Burnfield, 2010 (digitized by Carmichael Ong).
    soleus = controlsRef.updDependentColumn('soleus')
    gasmed = controlsRef.updDependentColumn('gastrocnemius')
    tibant = controlsRef.updDependentColumn('tibialis_anterior')
    for t in range(0, controlsRef.getNumRows()):
        soleus[t] = 0.77 * soleus[t]
        gasmed[t] = 0.87 * gasmed[t]
        tibant[t] = 0.37 * tibant[t]
    emgTracking.setReference(osim.TableProcessor(controlsRef))
    # Associate actuators in the model with columns in electromyography.sto.
    emgTracking.setReferenceLabel('/forceset/soleus_r', 'soleus')
    emgTracking.setReferenceLabel('/forceset/gasmed_r', 'gastrocnemius')
    emgTracking.setReferenceLabel('/forceset/gaslat_r', 'gastrocnemius')
    emgTracking.setReferenceLabel('/forceset/tibant_r', 'tibialis_anterior')
    problem.addGoal(emgTracking)

    # Solve the problem and write the solution to a Storage file.
    solution = study.solve()
    solution.write('example3DWalking_MocoInverseWithEMG_solution.sto')

    # Write the reference data in a way that's easy to compare to the solution.
    controlsRef.removeColumn('medial_hamstrings')
    controlsRef.removeColumn('biceps_femoris')
    controlsRef.removeColumn('vastus_lateralis')
    controlsRef.removeColumn('vastus_medius')
    controlsRef.removeColumn('rectus_femoris')
    controlsRef.removeColumn('gluteus_maximus')
    controlsRef.removeColumn('gluteus_medius')
    controlsRef.setColumnLabels(['/forceset/soleus_r', '/forceset/gasmed_r',
                                 '/forceset/tibant_r'])
    controlsRef.appendColumn('/forceset/gaslat_r', gasmed)
    osim.STOFileAdapter.write(controlsRef, 'controls_reference.sto')

    # Generate a report comparing MocoInverse solutions without and with EMG
    # tracking.
    model = modelProcessor.process()
    output = 'example3DWalking_MocoInverseWithEMG_report.pdf'
    ref_files = [
        'controls_reference.sto',
        'example3DWalking_MocoInverseWithEMG_solution.sto']
    report = osim.report.Report(model,
                                'example3DWalking_MocoInverse_solution.sto',
                                output=output, bilateral=True,
                                ref_files=ref_files,
                                colors=['black', 'blue', 'red'])
    # The PDF is saved to the working directory.
    report.generate()


# This problem solves in about 5 minutes.
solveMocoInverse()

# This problem penalizes the deviation from electromyography data for a
# subset of muscles, and solves in about 30 minutes.
solveMocoInverseWithEMG()
