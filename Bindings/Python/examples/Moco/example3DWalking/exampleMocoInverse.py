# -------------------------------------------------------------------------- #
# OpenSim Moco: exampleMocoInverse.py                                        #
# -------------------------------------------------------------------------- #
# Copyright (c) 2023 Stanford University and the Authors                     #
#                                                                            #
# Author(s): Christopher Dembia, Nicholas Bianco                             #
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
# from electromyography data for a subset of muscles. The third example
# extracts muscle synergies from the muscle excitaitons from the first example
# and uses them to solve the inverse problem using SynergyControllers.
#
# All examples use the Python utility osim.report to automatically generate a
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
    modelProcessor = osim.ModelProcessor('subject_walk_scaled.osim')
    modelProcessor.append(osim.ModOpAddExternalLoads('grf_walk.xml'))
    modelProcessor.append(osim.ModOpIgnoreTendonCompliance())
    modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
    # Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(osim.ModOpIgnorePassiveFiberForcesDGF())
    # Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
    # Use a function-based representation for the muscle paths. This is
    # recommended to speed up convergence, but if you would like to use
    # the original GeometryPath muscle wrapping instead, simply comment out
    # this line. To learn how to create a set of function-based paths for
    # your model, see the example 'examplePolynomialPathFitter.py'.
    modelProcessor.append(osim.ModOpReplacePathsWithFunctionBasedPaths(
        "subject_walk_scaled_FunctionBasedPathSet.xml"))
    modelProcessor.append(osim.ModOpAddReserves(1.0))
    inverse.setModel(modelProcessor)

    # Construct a TableProcessor of the coordinate data and pass it to the
    # inverse tool. TableProcessors can be used in the same way as
    # ModelProcessors by appending TableOperators to modify the base table.
    # A TableProcessor with no operators, as we have here, simply returns the
    # base table.
    inverse.setKinematics(osim.TableProcessor('coordinates.sto'))

    # Initial time, final time, and mesh interval.
    inverse.set_initial_time(0.48)
    inverse.set_final_time(1.61)
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
    modelProcessor = osim.ModelProcessor('subject_walk_scaled.osim')
    modelProcessor.append(osim.ModOpAddExternalLoads('grf_walk.xml'))
    modelProcessor.append(osim.ModOpIgnoreTendonCompliance())
    modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
    modelProcessor.append(osim.ModOpIgnorePassiveFiberForcesDGF())
    modelProcessor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
    modelProcessor.append(osim.ModOpReplacePathsWithFunctionBasedPaths(
        "subject_walk_scaled_FunctionBasedPathSet.xml"))
    modelProcessor.append(osim.ModOpAddReserves(1.0))
    inverse.setModel(modelProcessor)
    inverse.setKinematics(osim.TableProcessor('coordinates.sto'))
    inverse.set_initial_time(0.48)
    inverse.set_final_time(1.61)
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


# This problem extracts muscle synergies from the muscle excitations from the
# first example and uses them to solve the inverse problem using
# SynergyControllers.
def solveMocoInverseWithSynergies(numSynergies=5):

    # Construct the base model using a ModelProcessor as in the previous
    # examples, with the exception that we ignore activation dynamics to
    # simplify the problem given the muscle synergy controllers.
    modelProcessor = osim.ModelProcessor('subject_walk_scaled.osim')
    modelProcessor.append(osim.ModOpAddExternalLoads('grf_walk.xml'))
    modelProcessor.append(osim.ModOpIgnoreTendonCompliance())
    modelProcessor.append(osim.ModOpIgnoreActivationDynamics())
    modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
    modelProcessor.append(osim.ModOpIgnorePassiveFiberForcesDGF())
    modelProcessor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
    modelProcessor.append(osim.ModOpReplacePathsWithFunctionBasedPaths(
        "subject_walk_scaled_FunctionBasedPathSet.xml"))
    modelProcessor.append(osim.ModOpAddReserves(1.0))
    model = modelProcessor.process()

    # Load the solution from solveMocoInverse() to extract the muscle
    # control variable names and excitations for the left and right legs.
    prevSolution = osim.MocoTrajectory('example3DWalking_MocoInverse_solution.sto')
    leftControlNames = list()
    rightControlNames = list()
    model.initSystem()
    for actu in model.getComponentsList():
        if actu.getConcreteClassName().endswith('Muscle'):
            name = actu.getName()
            if name.endswith('_r'):
                rightControlNames.append(actu.getAbsolutePathString())
            elif name.endswith('_l'):
                leftControlNames.append(actu.getAbsolutePathString())
    
    controls = prevSolution.exportToControlsTable()
    leftControls = osim.TimeSeriesTable(controls.getIndependentColumn())
    rightControls = osim.TimeSeriesTable(controls.getIndependentColumn())
    for name in leftControlNames:
        leftControls.appendColumn(name, controls.getDependentColumn(name))
    for name in rightControlNames:
        rightControls.appendColumn(name, controls.getDependentColumn(name))

    # SynergyController
    # -----------------
    # SynergyController defines the controls for actuators connected to the 
    # controller using a linear combination of time-varying synergy control 
    # signals (i.e., "synergy excitations") and a set of vectors containing 
    # weights for each actuator representing the contribution of each synergy
    # excitation to the total control signal for that actuator 
    # (i.e., "synergy vectors").
    #
    # If 'N' is the number of time points in the trajectory, 'M' is the number
    # of actuators connected to the controller, and 'K' is the number of   
    # synergies in the controller, then:
    # - The synergy excitations are a matrix 'W' of size N x K.
    # - The synergy vectors are a matrix 'H' of size K x M.
    # - The controls for the actuators are computed A = W * H.
    #  
    # SynergyController is a concrete implementation of InputController, which
    # means that it uses Input control signals as the synergy excitations.
    # Moco automatically detects InputController%s in a model provided to a
    # MocoProblem and adds continuous variables to the optimization problem
    # for each Input control signal. The variable names are based on the path
    # to the controller appended with the Input control labels (e.g.,
    # "/path/to/my_synergy_controller/synergy_excitation_0").

    # Use non-negative matrix factorization (NNMF) to extract a set of muscle
    # synergies for each leg.
    from sklearn.decomposition import NMF
    import numpy as np
    nmf = NMF(n_components=numSynergies, init='random', random_state=0)
    
    Al = leftControls.getMatrix().to_numpy()
    Wl = nmf.fit_transform(Al)
    Hl = nmf.components_

    Ar = rightControls.getMatrix().to_numpy()
    Wr = nmf.fit_transform(Ar)
    Hr = nmf.components_

    # Scale W and H assuming that the elements of H are all 0.5. This prevents the 
    # synergy vector weights and synergy excitations from being very large or very
    # small.
    scaleVec = 0.5*np.ones(Hl.shape[1])
    for i in range(numSynergies):
        scale_l = np.linalg.norm(scaleVec) / np.linalg.norm(Hl[i, :])
        Hl[i, :] *= scale_l
        Wl[:, i] /= scale_l

        scale_r = np.linalg.norm(scaleVec) / np.linalg.norm(Hr[i, :])
        Hr[i, :] *= scale_r
        Wr[:, i] /= scale_r

    # Add a SynergyController for the left leg to the model.
    leftController = osim.SynergyController()
    leftController.setName("synergy_controller_left_leg")
    # The number of actuators connected to the controller defines the number of
    # weights in each synergy vector expected by the controller.
    for name in leftControlNames:
        leftController.addActuator(
                osim.Muscle.safeDownCast(model.getComponent(name)))
    # Adding a synergy vector increases the number of synergies in the
    # controller by one. This means that the number of Input control 
    # signals expected by the controller is also increased by one.
    for i in range(numSynergies):  
        synergyVector = osim.Vector(Hl.shape[1], 0.0)
        for j in range(Hl.shape[1]):
            synergyVector.set(j, Hl[i, j])
        leftController.addSynergyVector(synergyVector)
    model.addController(leftController)

    # Add a SynergyController for the right leg to the model.
    rightController = osim.SynergyController()
    rightController.setName("synergy_controller_right_leg")
    for name in rightControlNames:
        rightController.addActuator(
                osim.Muscle.safeDownCast(model.getComponent(name)))
    for i in range(numSynergies):
        synergyVector = osim.Vector(Hr.shape[1], 0.0)
        for j in range(Hr.shape[1]):
            synergyVector.set(j, Hr[i, j])
        rightController.addSynergyVector(synergyVector)
    model.addController(rightController)
    model.finalizeConnections()
    model.initSystem()

    # Construct the MocoInverse tool.
    inverse = osim.MocoInverse()
    inverse.setName("example3DWalking_MocoInverseWithSynergies")
    inverse.setModel(osim.ModelProcessor(model))
    inverse.setKinematics(osim.TableProcessor('coordinates.sto'))
    inverse.set_initial_time(0.48)
    inverse.set_final_time(1.61)
    inverse.set_mesh_interval(0.02)
    inverse.set_kinematics_allow_extra_columns(True)

    # Initialize the MocoInverse study and set the control bounds for the
    # muscle synergies excitations. 'setInputControlInfo()' is equivalent to
    # 'setControlInfo()', but reserved for Input control variables. Note that 
    # while we make a distinction between "control" variables and 
    # "Input control" variables in the API, the optimal control problem
    # constructed by Moco treats them both as algebraic variables.
    study = inverse.initialize()
    problem = study.updProblem()

    # We will also increase the weight on the synergy excitations in the 
    # control effort cost term. MocoControlGoal, and other MocoGoals, that 
    # depend on control variables have options configuring cost terms with
    # Input control values.
    effort = osim.MocoControlGoal.safeDownCast(
            problem.updGoal("excitation_effort"))
    for i in range(numSynergies):
        nameLeft = (f'/controllerset/synergy_controller_left_leg' 
                    f'/synergy_excitation_{i}')
        problem.setInputControlInfo(nameLeft, [0, 1.0])
        effort.setWeightForControl(nameLeft, 10)

        nameRight = (f'/controllerset/synergy_controller_right_leg'
                     f'/synergy_excitation_{i}')
        problem.setInputControlInfo(nameRight, [0, 1.0])
        effort.setWeightForControl(nameRight, 10)

    # Solve!
    solution = study.solve()

    # This function computes the model control values from the
    # SynergyControllers in the model and inserts them into the solution.
    solution.generateControlsFromModelControllers(model)

    # Add the multibody states into the solutions so we can visualize the
    # trajectory or utilize the plotting utilities.
    coordinateValues = prevSolution.exportToValuesTable()
    coordinateSpeeds = prevSolution.exportToSpeedsTable()
    solution.insertStatesTrajectory(coordinateValues)
    solution.insertStatesTrajectory(coordinateSpeeds)

    # Write the solution to a Storage file.
    solutionFile = (f'example3DWalking_MocoInverseWith'
                    f'{numSynergies}Synergies_solution.sto')
    solution.write(solutionFile)
    
    # Generate a report comparing MocoInverse solutions with and without
    # muscle synergies.
    output = (f'example3DWalking_MocoInverseWith'
             f'{numSynergies}Synergies_report.pdf')
    ref_files = ['example3DWalking_MocoInverse_solution.sto']
    report = osim.report.Report(model, solutionFile,
                                output=output, bilateral=True,
                                ref_files=ref_files,
                                colors=['black', 'red'])
    # The PDF is saved to the working directory.
    report.generate()
    

# Solve the basic muscle redundancy problem with MocoInverse.
solveMocoInverse()

# This problem penalizes the deviation from electromyography data for a
# subset of muscles.
solveMocoInverseWithEMG()

# This problem extracts muscle synergies from the muscle excitations from
# the first example and uses them to solve the inverse problem using
# SynergyControllers.
numSynergies = 5
solveMocoInverseWithSynergies(numSynergies)
